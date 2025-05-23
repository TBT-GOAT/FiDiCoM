
#ifndef UNION_FIND_H
#define UNION_FIND_H

// include std libraries
#include <unordered_map>
#include <unordered_set>
#include <vector>

template <typename T>
class Union_Find {
    private:
        std::unordered_map<T, T> parent;
    
    public:
        //** Constructor **//
        Union_Find() = default;

        //** Main Procedure **//
        /*************************************************
         * @brief 初期化
         * 
         * @param x 
         *************************************************/
        void initialize(const T& x) {
            if (parent.find(x) == parent.end()) {
                parent[x] = x;
            }
        }
        /*************************************************
         * @brief 入力した要素の（集合を表す）親を判別する
         * 
         * @param x 
         * @return T 
         *************************************************/
        T find(const T& x) {
            initialize(x);
            if (parent[x] != x) {
                // 根の要素で親を統一する
                parent[x] = find(parent[x]);
            }
            return parent[x];
        }
        /*************************************************
         * @brief 異なる要素が属する集合を結合する
         * 
         * @param x 
         * @param y 
         *************************************************/
        void unite(const T& x, const T& y) {
            T root_x = find(x);
            T root_y = find(y);
            if (root_x != root_y) {
                // yの親をxの親にする
                parent[root_y] = root_x;
            }
        }
        /*************************************************
         * @brief ふたつの要素が同じ集合にあるかどうかを判別する
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         *************************************************/
        bool is_connected(const T&x, const T& y) {
            return find(x) == find(y);
        }
        /*************************************************
         * @brief 集合の数を数え上げる
         * 
         * @param elements 
         * @return size_t 
         *************************************************/
        size_t count_components(const std::vector<T>& elements) {
            std::unordered_set<T> roots;
            for (const auto& x : elements) {
                roots.insert(find(x));
            }
            return roots.size();
        }
};

#endif // UNION_FIND_H