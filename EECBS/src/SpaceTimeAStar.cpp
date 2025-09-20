#include "SpaceTimeAStar.h"


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    const LLNode* curr = goal;
    if (curr->is_goal)
        curr = curr->parent;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr)
    {
        // ★ 3D → 2D に射影して保存（可視化/上位は2D想定）
        path.emplace_back(instance.base2D(curr->location));
        curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());
}


Path SpaceTimeAStar::findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
                                     const vector<Path*>& paths, int agent, int lowerbound)
{
    return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
}

// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
pair<Path, int> SpaceTimeAStar::findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
                                                   const vector<Path*>& paths, int agent, int lowerbound, double w)
{
    this->w = w;
    Path path;
    num_expanded = 0;
    num_generated = 0;

    // --- 制約テーブル構築 ---
    auto t = clock();
    ConstraintTable constraint_table(initial_constraints);
    constraint_table.insert2CT(node, agent);
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // ★ startの制約（2D）を先に見る
    if (constraint_table.constrained(start_location, 0))
        return {path, 0};

    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // --- ゴール保持時刻（2Dゴール） ---
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    auto static_timestep = constraint_table.getMaxTimestep() + 1;
    lowerbound = max(holding_time, lowerbound);

    // --- ★ start を 3D にエンコードして開始（z=0 から） ---
    const int sr = instance.getRowCoordinate(start_location);
    const int sc = instance.getColCoordinate(start_location);
    const int start3 = instance.encode3D(sr, sc, 0);

    auto start = new AStarNode(
        start3,
        0,
        max(lowerbound, my_heuristic[start_location]), // h は 2D 配列
        nullptr, 0, 0);

    num_generated++;
    start->open_handle  = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist  = true;
    allNodes_table.insert(start);
    min_f_val = (int) start->getFVal();

    while (!open_list.empty())
    {
        updateFocalList();
        auto* curr = popNode();
        assert(curr->location >= 0);

        // --- ★ ゴール判定は 2D 射影で ---
        if (instance.base2D(curr->location) == goal_location &&
            !curr->wait_at_goal &&
            curr->timestep >= holding_time)
        {
            updatePath(curr, path);
            break;
        }

        if (curr->timestep >= constraint_table.length_max)
            continue;

        // --- ★ 近傍は 3D で展開、待機は別途許可 ---
        auto next_locations = instance.getNeighbors3D(curr->location);
        next_locations.emplace_back(curr->location);

        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            {   // 静的相転移後は同時刻扱い（元実装の仕様に合わせる）
                if (next_location == curr->location)
                    continue;
                next_timestep--;
            }

            // --- ★ 制約/衝突は 2D 射影で判定 ---
            const int base_curr = instance.base2D(curr->location);
            const int base_next = instance.base2D(next_location);

            if (constraint_table.constrained(base_next, next_timestep) ||
                constraint_table.constrained(base_curr, base_next, next_timestep))
                continue;

            // --- g/h/conflicts ---
            int next_g_val = curr->g_val + 1;
            int next_h_val = max(lowerbound - next_g_val, my_heuristic[base_next]); // ★ h は 2D index
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;

            int next_internal_conflicts =
                curr->num_of_conflicts +
                constraint_table.getNumOfConflictsForStep(base_curr, base_next, next_timestep);

            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, next_internal_conflicts);

            // ゴール上待機フラグ（2Dで同一点なら true）
            if (base_next == goal_location && base_curr == goal_location)
                next->wait_at_goal = true;

            // 既存ノードの更新処理（元実装に準拠）
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }
            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal() ||
                (existing_next->getFVal() == next->getFVal() &&
                 existing_next->num_of_conflicts > next->num_of_conflicts))
            {
                if (!existing_next->in_openlist)
                {
                    existing_next->copy(*next);
                    pushNode(existing_next);
                }
                else
                {
                    bool add_to_focal     = false;
                    bool update_in_focal  = false;
                    bool update_open      = false;
                    if ((next_g_val + next_h_val) <= w * min_f_val)
                    {
                        if (existing_next->getFVal() > w * min_f_val)
                            add_to_focal = true;
                        else
                            update_in_focal = true;
                    }
                    if (existing_next->getFVal() > next_g_val + next_h_val)
                        update_open = true;

                    existing_next->copy(*next);

                    if (update_open)
                        open_list.increase(existing_next->open_handle);
                    if (add_to_focal)
                        existing_next->focal_handle = focal_list.push(existing_next);
                    if (update_in_focal)
                        focal_list.update(existing_next->focal_handle);
                }
            }
            delete next;
        }
    }

    releaseNodes();
    return {path, min_f_val};
}


int SpaceTimeAStar::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
    int length = MAX_TIMESTEP;
    auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep
    auto root = new AStarNode(start, 0, compute_heuristic(start, end), nullptr, 0, 0);
    root->open_handle = open_list.push(root);  // add root to heap
    allNodes_table.insert(root);       // add root to hash_table (nodes)
    AStarNode* curr = nullptr;
    while (!open_list.empty())
    {
        curr = open_list.top(); open_list.pop();
        if (curr->location == end)
        {
            length = curr->g_val;
            break;
        }
        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            int next_g_val = curr->g_val + 1;
            if (static_timestep < next_timestep)
            {
                if (curr->location == next_location)
                {
                    continue;
                }
                next_timestep--;
            }
            if (!constraint_table.constrained(next_location, next_timestep) &&
                !constraint_table.constrained(curr->location, next_location, next_timestep))
            {  // if that grid is not blocked
                int next_h_val = compute_heuristic(next_location, end);
                if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
                    continue;
                auto next = new AStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, 0);
                auto it = allNodes_table.find(next);
                if (it == allNodes_table.end())
                {  // add the newly generated node to heap and hash table
                    next->open_handle = open_list.push(next);
                    allNodes_table.insert(next);
                }
                else {  // update existing node's g_val if needed (only in the heap)
                    delete(next);  // not needed anymore -- we already generated it before
                    auto existing_next = *it;
                    if (existing_next->g_val > next_g_val)
                    {
                        existing_next->g_val = next_g_val;
                        existing_next->timestep = next_timestep;
                        open_list.increase(existing_next->open_handle);
                    }
                }
            }
        }
    }
    releaseNodes();
    return length;
}

inline AStarNode* SpaceTimeAStar::popNode()
{
    auto node = focal_list.top(); focal_list.pop();
    open_list.erase(node->open_handle);
    node->in_openlist = false;
    num_expanded++;
    return node;
}


inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
}


void SpaceTimeAStar::updateFocalList()
{
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val)
    {
        int new_min_f_val = (int)open_head->getFVal();
        for (auto n : open_list)
        {
            if (n->getFVal() >  w * min_f_val && n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}


void SpaceTimeAStar::releaseNodes()
{
    open_list.clear();
    focal_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
}
