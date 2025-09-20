#pragma once
#include"common.h"

// 現行定義はそのままにしてOK

class Instance 
{
public:
    int num_of_cols;
    int num_of_rows;
    int map_size;

    // === ここから追加: 3D（z層）サポート ===
    // 回転フェーズ段数（=回転1回が直進5回ぶんのコスト）
    static constexpr int WEIGHT = 5;

    // 2D 平面サイズ（cols * rows）
    inline int baseSize() const { return num_of_cols * num_of_rows; }

    // 3D ロケーションID <-> 2D/3D 座標の相互変換
    inline int encode3D(int row, int col, int z) const { return z * baseSize() + linearizeCoordinate(row, col); }
    inline int base2D(int loc3) const { return loc3 % baseSize(); } // 3D ID -> 2D ID（(x,y) 射影）

    inline void decode3D(int loc3, int& row, int& col, int& z) const {
        z = loc3 / baseSize();
        int base = loc3 % baseSize();
        row = getRowCoordinate(base);
        col = getColCoordinate(base);
    }

    // 3D 近傍: 回転フェーズ z±1（回転に時間を使う）＋
    //   z==0 のとき縦（row±1）に前進可、z==WEIGHT-1 のとき横（col±1）に前進可。
    //   ※ 2D API は互換のため変更しません（他コードが使っているため）。
    std::list<int> getNeighbors3D(int curr3) const;
    // === ここまで追加 ===

    Instance(){}
    Instance(const string& map_fname, const string& agent_fname, 
        int num_of_agents = 0, int num_of_rows = 0, int num_of_cols = 0, int num_of_obstacles = 0, int warehouse_width = 0);

    void printAgents() const;

    inline bool isObstacle(int loc) const { return my_map[loc]; }
    inline bool validMove(int curr, int next) const;
    std::list<int> getNeighbors(int curr) const;

    inline int linearizeCoordinate(int row, int col) const { return ( this->num_of_cols * row + col); }
    inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
    inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
    inline std::pair<int, int> getCoordinate(int id) const { return std::make_pair(id / this->num_of_cols, id % this->num_of_cols); }
    inline int getCols() const { return num_of_cols; }

    inline int getManhattanDistance(int loc1, int loc2) const
    {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    inline int getManhattanDistance(const std::pair<int, int>& loc1, const std::pair<int, int>& loc2) const
    {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

    int getDegree(int loc) const
    {
        assert(loc >= 0 && loc < map_size && !my_map[loc]);
        int degree = 0;
        if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
            degree++;
        if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
            degree++;
        if (loc % num_of_cols > 0 && !my_map[loc - 1])
            degree++;
        if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
            degree++;
        return degree;
    }

    int getDefaultNumberOfAgents() const { return num_of_agents; }

private:
    vector<bool> my_map;
    string map_fname;
    string agent_fname;

    int num_of_agents;
    vector<int> start_locations;
    vector<int> goal_locations;

    bool loadMap();
    void printMap() const;
    void saveMap() const;

    bool loadAgents();
    void saveAgents() const;

    void generateConnectedRandomGrid(int rows, int cols, int obstacles);
    void generateRandomAgents(int warehouse_width);
    bool addObstacle(int obstacle);
    bool isConnected(int start, int goal);

    int randomWalk(int loc, int steps) const;

    friend class SingleAgentSolver;
};