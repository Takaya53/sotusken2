#include "SingleAgentSolver.h"


std::list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
    // ★ 2D 近傍（MDD は 2D しか扱わない）
    std::list<int> rst = instance.getNeighbors(curr);
    // 待機も許可
    rst.emplace_back(curr);
    return rst;
}


void SingleAgentSolver::compute_heuristics()
{
    struct Node
    {
        int location;
        int value;

        Node() = default;
        Node(int location, int value) : location(location), value(value) {}
        struct compare_node
        {
            bool operator()(const Node& n1, const Node& n2) const
            {
                return n1.value >= n2.value;
            }
        };
    };

    my_heuristic.resize(instance.map_size, MAX_TIMESTEP);

    boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node>> heap;

    Node root(goal_location, 0);
    my_heuristic[goal_location] = 0;
    heap.push(root);
    while (!heap.empty())
    {
        Node curr = heap.top();
        heap.pop();
        // ★ ヒューリスティックは 2D 近傍でOK（配列も2Dサイズのため）
        for (int next_location : instance.getNeighbors(curr.location))
        {
            if (my_heuristic[next_location] > curr.value + 1)
            {
                my_heuristic[next_location] = curr.value + 1;
                Node next(next_location, curr.value + 1);
                heap.push(next);
            }
        }
    }
}