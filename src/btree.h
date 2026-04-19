#ifndef BTREE_H
#define BTREE_H

#include <vector>
#include <cstdlib>
#include <climits>
#include <algorithm>
#include "module.h"

struct BTreeNode {
    int blockIdx;
    int parent, left, right;
    BTreeNode() : blockIdx(-1), parent(-1), left(-1), right(-1) {}
};

class BTree {
public:
    BTree() : _root(-1) {}

    void init(int n) {
        _nodes.resize(n);
        for (int i = 0; i < n; i++) {
            _nodes[i].blockIdx = i;
            _nodes[i].parent = -1;
            _nodes[i].left = -1;
            _nodes[i].right = -1;
        }
        // Build initial left-skewed tree
        _root = 0;
        for (int i = 1; i < n; i++) {
            _nodes[i - 1].left = i;
            _nodes[i].parent = i - 1;
        }
    }

    void pack(vector<Block*>& blocks, int& totalW, int& totalH) {
        _contour.clear();
        _contour.push_back({0, 0, INT_MAX, 0}); // sentinel
        totalW = 0; totalH = 0;
        if (_root >= 0)
            dfs(blocks, _root, 0, totalW, totalH);
    }

    int getRoot() const { return _root; }
    vector<BTreeNode>& getNodes() { return _nodes; }

    // Perturbation operations
    void rotate(int idx) {
        // handled externally by toggling block rotation
    }

    void deleteAndInsert(int delIdx, int insIdx, bool toLeft) {
        deleteNode(delIdx);
        insertNode(delIdx, insIdx, toLeft);
    }

    void swapNodes(int a, int b) {
        // Just swap block indices
        swap(_nodes[a].blockIdx, _nodes[b].blockIdx);
    }

    // Save/restore for SA
    vector<BTreeNode> saveNodes() const { return _nodes; }
    int saveRoot() const { return _root; }
    void restore(const vector<BTreeNode>& nodes, int root) {
        _nodes = nodes;
        _root = root;
    }

private:
    struct ContourSeg {
        int x1;
        int y_top;
        int x2;
        int y_top_val;
        ContourSeg(int a, int b, int c, int d) : x1(a), y_top(b), x2(c), y_top_val(d) {}
    };

    int _root;
    vector<BTreeNode> _nodes;
    vector<ContourSeg> _contour;

    void dfs(vector<Block*>& blocks, int nodeIdx, int x, int& totalW, int& totalH) {
        int bIdx = _nodes[nodeIdx].blockIdx;
        Block* blk = blocks[bIdx];
        int w = blk->getWidth();
        int h = blk->getHeight();

        // Find max y on contour in [x, x+w)
        int y = getContourMaxY(x, x + w);
        blk->setPos(x, y, x + w, y + h);
        updateContour(x, x + w, y + h);

        if (x + w > totalW) totalW = x + w;
        if (y + h > totalH) totalH = y + h;

        // Left child: placed at (x + w, ...)
        if (_nodes[nodeIdx].left >= 0)
            dfs(blocks, _nodes[nodeIdx].left, x + w, totalW, totalH);
        // Right child: placed at (x, ...) -- same x as parent
        if (_nodes[nodeIdx].right >= 0)
            dfs(blocks, _nodes[nodeIdx].right, x, totalW, totalH);
    }

    int getContourMaxY(int x1, int x2) {
        int maxY = 0;
        for (auto& seg : _contour) {
            if (seg.x1 < x2 && seg.x2 > x1) {
                maxY = max(maxY, seg.y_top_val);
            }
        }
        return maxY;
    }

    void updateContour(int x1, int x2, int y) {
        // Remove segments fully covered
        vector<ContourSeg> newContour;
        for (auto& seg : _contour) {
            if (seg.x2 <= x1 || seg.x1 >= x2) {
                newContour.push_back(seg);
            } else {
                // Partially overlapping
                if (seg.x1 < x1)
                    newContour.push_back({seg.x1, seg.y_top, x1, seg.y_top_val});
                if (seg.x2 > x2)
                    newContour.push_back({x2, seg.y_top, seg.x2, seg.y_top_val});
            }
        }
        newContour.push_back({x1, y, x2, y});
        _contour = newContour;
    }

    void deleteNode(int idx) {
        int p = _nodes[idx].parent;
        int l = _nodes[idx].left;
        int r = _nodes[idx].right;

        int child = -1;
        if (l == -1 && r == -1) {
            child = -1;
        } else if (l == -1) {
            child = r;
        } else if (r == -1) {
            child = l;
        } else {
            // Has both children: pull up one subtree
            // Attach right subtree to the rightmost node of left subtree
            child = l;
            int rightmost = l;
            while (_nodes[rightmost].right != -1)
                rightmost = _nodes[rightmost].right;
            _nodes[rightmost].right = r;
            _nodes[r].parent = rightmost;
        }

        if (child != -1) _nodes[child].parent = p;

        if (p == -1) {
            _root = child;
        } else {
            if (_nodes[p].left == idx) _nodes[p].left = child;
            else _nodes[p].right = child;
        }

        _nodes[idx].parent = -1;
        _nodes[idx].left = -1;
        _nodes[idx].right = -1;
    }

    void insertNode(int idx, int target, bool toLeft) {
        if (target == -1) {
            // Insert as root
            _nodes[idx].left = _root;
            if (_root != -1) _nodes[_root].parent = idx;
            _root = idx;
            _nodes[idx].parent = -1;
            return;
        }
        if (toLeft) {
            int old = _nodes[target].left;
            _nodes[target].left = idx;
            _nodes[idx].parent = target;
            _nodes[idx].left = old;
            if (old != -1) _nodes[old].parent = idx;
        } else {
            int old = _nodes[target].right;
            _nodes[target].right = idx;
            _nodes[idx].parent = target;
            _nodes[idx].right = old;
            if (old != -1) _nodes[old].parent = idx;
        }
    }
};

#endif
