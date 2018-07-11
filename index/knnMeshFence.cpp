#include "knnMeshFence.h"
#include "expansion.h"
#include "point.h"

using namespace std;

namespace polyanya {

int KnnMeshEdgeFence::succ_to_node(SearchNodePtr parent, Successor* successors, int num_succ, SearchNodePtr nodes, int gid) {

  assert(mesh != nullptr);
  const Polygon& polygon = mesh->mesh_polygons[parent->next_polygon];
  const std::vector<int>& V = polygon.vertices;
  const std::vector<int>& P = polygon.polygons;
  const Point& start = goals[gid];

  double right_g = -1, left_g = -1;
  int out = 0;
  for (int i = 0; i < num_succ; i++) {
    const Successor& succ = successors[i];
    const int next_polygon = P[succ.poly_left_ind];
    if (next_polygon == -1) continue;
    const int left_vertex = V[succ.poly_left_ind];
    const int right_vertex = succ.poly_left_ind? V[succ.poly_left_ind - 1]: V.back();

    // We implicitly set h to be zero and let search() update it.
    const auto process = [&](const int root, const double g) {
      if (root != -1) {
        assert(root >= 0 && root < (int) root_g_values.size());
        if (root_search_ids[root] != search_id) {
          // first time reaching root
          root_search_ids[root] = search_id;
          root_g_values[root] = g;
        } else {
          // visited
          if (root_g_values[root] + EPSILON < g) return;
          else root_g_values[root] = g;
        }
      }
      nodes[out++] = {nullptr, root, succ.left, succ.right, left_vertex, right_vertex, next_polygon, g, g};
      nodes[out-1].heuristic_gid = parent->heuristic_gid;
    };

    const Point& parent_root =  parent->root == -1? start: mesh->mesh_vertices[parent->root].p;
    #define get_g(new_root) parent->g + parent_root.distance(new_root)

    switch (succ.type) {
      case Successor::RIGHT_NON_OBSERVABLE:
        if (right_g == -1) right_g = get_g(parent->right);
        process(parent->right_vertex, right_g);
        break;

      case Successor::OBSERVABLE:
        process(parent->root, parent->g);
        break;

      case Successor::LEFT_NON_OBSERVABLE:
        if (left_g == -1) left_g = get_g(parent->left);
        process(parent->left_vertex, left_g);
        break;

      default:
        assert(false);
        break;
    }
  }
  #undef get_g
  return out;
}

void KnnMeshEdgeFence::gen_initial_nodes() {
  #define get_lazy(next, left, right, gid) new (node_pool->allocate()) SearchNode \
  {nullptr, -1, goals[gid], goals[gid], left, right, next, 0, 0}
  #define v(vertex) mesh->mesh_vertices[vertex]
  const auto push_lazy = [&](SearchNodePtr lazy, int gid) {
    const int poly = lazy->next_polygon;
    if (poly == -1) return;

    const std::vector<int>& vertices = mesh->mesh_polygons[poly].vertices;
    Successor* successors = new Successor [vertices.size()];
    int last_vertex = vertices.back();
    int num_succ = 0;
    for (int i = 0; i < (int) vertices.size(); i++) {
      const int vertex = vertices[i];
      if (vertex == lazy->right_vertex ||
          last_vertex == lazy->left_vertex) {
        last_vertex = vertex;
        continue;
      }
      successors[num_succ++] = {Successor::OBSERVABLE, v(vertex).p, v(last_vertex).p, i};
      last_vertex = vertex;
    }
    SearchNode* nodes = new SearchNode [num_succ];
    const int num_nodes = succ_to_node(lazy, successors, num_succ, nodes, gid);

    const Point& start = goals[gid];
    for (int i = 0; i < num_nodes; i++) {
      SearchNodePtr nxt = new (node_pool->allocate()) SearchNode(nodes[i]);
      const Point& nxt_root = nxt->root == -1? start: mesh->mesh_vertices[nxt->root].p;
      nxt->f += get_interval_heuristic(nxt_root, nxt->left, nxt->right);
      nxt->parent = lazy;
      const Point& left = mesh->mesh_vertices[nxt->left_vertex].p;
      const Point& right = mesh->mesh_vertices[nxt->right_vertex].p;
      double lb = nxt->f;
      double ub = nxt->g + max(nxt_root.distance(left), nxt_root.distance(right));
      FloodFillNode fnode(nxt, lb, ub, gid, poly, successors[i].poly_left_ind);
      #ifndef NDEBUG
      if (verbose) {
        std::cerr << "generating init node: ";
        print_node(fnode, std::cerr);
        std::cerr << std::endl;
      }
      #endif
      open_list.push(fnode);
      nodes_pushed++;
    }
    delete[] nodes;
    delete[] successors;
    nodes_generated += num_nodes;
    nodes_pushed += num_nodes;
  };
  for (int i=0; i<(int)goals.size(); i++) {
    const PointLocation pl = get_point_location_in_search(goals[i], mesh, verbose);
    switch(pl.type) {
      case PointLocation::NOT_ON_MESH:
        break;
      case PointLocation::ON_CORNER_VERTEX_AMBIG:
        if (pl.poly1 == -1) break;
      case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
      case PointLocation::IN_POLYGON:
        {
          SearchNodePtr lazy = get_lazy(pl.poly1, -1, -1, i);
          push_lazy(lazy, i);
          nodes_generated++;
        }
        break;
      case PointLocation::ON_MESH_BORDER:
        {
          SearchNodePtr lazy = get_lazy(pl.poly1, -1, -1, i);
          push_lazy(lazy, i);
          nodes_generated++;
        }
        break;
      case PointLocation::ON_EDGE:
        {
          SearchNodePtr lazy1 = get_lazy(pl.poly2, pl.vertex1, pl.vertex2, i);
          SearchNodePtr lazy2 = get_lazy(pl.poly1, pl.vertex2, pl.vertex1, i);
          push_lazy(lazy1, i);
          nodes_generated++;
          push_lazy(lazy2, i);
          nodes_generated++;
        }
        break;
      case PointLocation::ON_NON_CORNER_VERTEX:
        {
          for (int& poly: v(pl.vertex1).polygons) {
            SearchNodePtr lazy = get_lazy(poly, pl.vertex1, pl.vertex1, i);
            push_lazy(lazy, i);
            nodes_generated++;
          }
        }
        break;

      default:
        assert(false);
        break;
    }
  }
}

void KnnMeshEdgeFence::floodfill() {
  assert(mesh != nullptr);
  init_floodfill();
  timer.start();

  while (!open_list.empty()) {
    FloodFillNode fnode = open_list.top(); open_list.pop();
    SearchNodePtr snode = fnode.snode;

    if (!pass_fence(fnode))
      continue;

    Point start = goals[fnode.gid];

    #ifndef NDEBUG
    if (verbose) {
      std::cerr << "popped off: ";
      print_node(fnode, std::cerr);
      std::cerr << std::endl;
    }
    #endif

    nodes_popped++;
    const int root = snode->root;
    if (root != -1) {
      assert(root < (int) root_g_values.size());
      if (root_search_ids[root] == search_id) {
        if (root_g_values[root] + EPSILON < snode->g) {
            nodes_pruned++;
            #ifndef NDEBUG
            if (verbose) std::cerr << "node is dominated!" << std::endl;
            #endif
            continue;
        }
      }
    }
    search_nodes_to_push[0] = *snode;
    SearchNode cur = search_nodes_to_push[0];
    int num_succ = get_successors(search_nodes_to_push[0], start, *mesh, search_successors);
    int num_nodes = succ_to_node(&cur, search_successors, num_succ, search_nodes_to_push, fnode.gid);
    for (int i=0; i<num_nodes; i++) {
      const SearchNodePtr nxt = new (node_pool->allocate()) SearchNode(search_nodes_to_push[i]);
      const Point& nxt_root = (nxt->root == -1? start: mesh->mesh_vertices[nxt->root].p);
      nxt->f += get_interval_heuristic(nxt_root, nxt->left, nxt->right);
      nxt->parent = snode;
      const Point& left = mesh->mesh_vertices[nxt->left_vertex].p;
      const Point& right = mesh->mesh_vertices[nxt->right_vertex].p;
      double lb = nxt->f;
      double ub = nxt->g + max(nxt_root.distance(nxt->left) + nxt->left.distance(left), nxt_root.distance(nxt->right) + nxt->right.distance(right));
      FloodFillNode nxt_fnode = FloodFillNode(nxt, lb, ub, fnode.gid,
          snode->next_polygon, search_successors[i].poly_left_ind);

      #ifndef NDEBUG
      if (verbose) {
        std::cerr << "\tpushing: ";
        print_node(nxt_fnode, std::cerr);
        std::cerr << std::endl;
      }
      #endif
      open_list.push(nxt_fnode);
      nodes_generated++;
    }
  }
  timer.stop();
}

bool KnnMeshEdgeFence::pass_fence(const FloodFillNode& fnode) {
  int left_vid = fnode.snode->left_vertex;
  int right_vid = fnode.snode->right_vertex;
  pair<int, int> key = {min(left_vid, right_vid), max(left_vid, right_vid)};
  if (fences[key].empty()) {
    Fence fence(fnode.lb, fnode.ub, fnode.gid, fnode.snode);
    fences[key].push_back(fence);
    fenceCnt++;
    return true;
  }
  else {
    assert(fences[key][0].lb <= fnode.lb);
    if (fnode.lb <= fences[key][0].ub) {
      fences[key][0].ub = min(fnode.ub, fences[key][0].ub);
      Fence fence(fnode.lb, fnode.ub, fnode.gid, fnode.snode);
      fences[key].push_back(fence);
      fenceCnt++;
      return true;
    }
  }
  return false;
}

void KnnMeshEdgeFence::print_node(const FloodFillNode& fnode, ostream& outfile) {
  const Point& root = fnode.snode->root == -1? goals[fnode.gid]: mesh->mesh_vertices[fnode.snode->root].p;
  outfile << "root=" << root << "; left=" << fnode.snode->left
          << "; right=" << fnode.snode->right << "; f=" << fnode.snode->f << ", g="
          << fnode.snode->g << ", lb=" << fnode.lb << ", ub=" << fnode.ub
          << ", lid=" << fnode.snode->left_vertex << ", rid=" << fnode.snode->right_vertex
          <<", gid=" << fnode.gid;
}

} // end polyanya namespace
