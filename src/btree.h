#ifndef BTREE_H
#define BTREE_H

#include <vector>
#include <utility>
#include <cstdlib>
#include <climits>
#include <algorithm>
#include "module.h"

struct BTreeNode {
    int blockIdx;
    int parent, left, right;
    BTreeNode() : blockIdx(-1), parent(-1), left(-1), right(-1) {}
};

// ---------------------------------------------------------------------------
// Horizontal contour represented as a doubly-linked list of segments.
//
// Each ContourNode covers the half-open interval [x, next->x) at height y.
// The list is bookended by two fixed sentinels:
//   head : x = 0,       y = 0
//   tail : x = INT_MAX, y = 0
//
// Amortised complexity per pack():
//   scanContour and updateContour together traverse each segment at most once.
//   Every ContourNode is created once and removed at most once per pack().
//   Total work for all n blocks = O(n).  Amortised O(1) per block.
//
// Pool allocator: nodes come from a pre-sized vector so there are zero heap
// allocations during DFS.  The pool index is simply reset at pack() start.
// ---------------------------------------------------------------------------
class BTree {
public:
    BTree() : _root(-1), _lastW(0), _lastH(0), _poolIdx(0) {}

    void init(int n) {
        _nodes.resize(n);
        for (int i = 0; i < n; i++) {
            _nodes[i].blockIdx = i;
            _nodes[i].parent   = -1;
            _nodes[i].left     = -1;
            _nodes[i].right    = -1;
        }
        // Build initial left-skewed tree (horizontal strip layout)
        _root = 0;
        for (int i = 1; i < n; i++) {
            _nodes[i - 1].left = i;
            _nodes[i].parent   = i - 1;
        }
        // Each block placement allocates at most 2 ContourNodes.
        // Add 2 for the head/tail sentinels created at pack() start.
        _pool.resize(2 * n + 4);
    }

    // Build a random binary tree with shuffled block order.
    void randomly_init(int n) {
        _nodes.resize(n);
        // Assign a random permutation of block indices
        vector<int> perm(n);
        for (int i = 0; i < n; i++) perm[i] = i;
        for (int i = n - 1; i > 0; i--) {
            int j = rand() % (i + 1);
            swap(perm[i], perm[j]);
        }
        for (int i = 0; i < n; i++) {
            _nodes[i].blockIdx = perm[i];
            _nodes[i].parent   = -1;
            _nodes[i].left     = -1;
            _nodes[i].right    = -1;
        }
        // Build a random tree by inserting nodes one by one
        _root = 0;
        for (int i = 1; i < n; i++) {
            // Pick a random existing node and a random side
            int target = rand() % i;
            bool goLeft = rand() % 2;
            // Walk down until we find a free slot on that side
            while (true) {
                if (goLeft) {
                    if (_nodes[target].left == -1) {
                        _nodes[target].left = i;
                        _nodes[i].parent = target;
                        break;
                    }
                    target = _nodes[target].left;
                    goLeft = rand() % 2;
                } else {
                    if (_nodes[target].right == -1) {
                        _nodes[target].right = i;
                        _nodes[i].parent = target;
                        break;
                    }
                    target = _nodes[target].right;
                    goLeft = rand() % 2;
                }
            }
        }
        _pool.resize(2 * n + 4);
    }

    void pack(vector<Block*>& blocks, int& totalW, int& totalH) {
        // Reset pool and create sentinels
        _poolIdx = 0;
        ContourNode* head = allocNode(0,       0);
        ContourNode* tail = allocNode(INT_MAX, 0);
        head->next = tail;  tail->prev = head;
        head->prev = nullptr; tail->next = nullptr;

        totalW = 0; totalH = 0;
        if (_root >= 0)
            dfs(blocks, _root, 0, head, totalW, totalH);
        _lastW = totalW;
        _lastH = totalH;
    }

    int  getRoot()  const { return _root;  }
    int  getLastW() const { return _lastW; }
    int  getLastH() const { return _lastH; }
    vector<BTreeNode>& getNodes() { return _nodes; }

    // Copy tree state into a caller-supplied pre-allocated buffer (no heap).
    void saveState(vector<BTreeNode>& outNodes, int& outRoot) const {
        outNodes = _nodes;
        outRoot  = _root;
    }
    // Legacy helpers kept for compatibility with floorplanner.cpp
    vector<BTreeNode> saveNodes() const { return _nodes; }
    int               saveRoot()  const { return _root;  }
    void restore(const vector<BTreeNode>& nodes, int root) {
        _nodes = nodes;
        _root  = root;
    }

    // ── Perturbation operations ──────────────────────────────────────────
    void deleteAndInsert(int delIdx, int insIdx, bool toLeft) {
        deleteNode(delIdx);
        insertNode(delIdx, insIdx, toLeft);
    }
    void swapNodes(int a, int b) {
        swap(_nodes[a].blockIdx, _nodes[b].blockIdx);
    }

private:
    // ── Doubly-linked contour node ───────────────────────────────────────
    struct ContourNode {
        int x, y;            // segment starts at x with height y
        ContourNode* prev;
        ContourNode* next;
        ContourNode() : x(0), y(0), prev(nullptr), next(nullptr) {}
    };

    int _root, _lastW, _lastH;
    vector<BTreeNode>   _nodes;
    vector<ContourNode> _pool;   // pre-allocated pool — no new/delete in DFS
    int _poolIdx;

    ContourNode* allocNode(int x, int y) {
        ContourNode* p = &_pool[_poolIdx++];
        p->x = x;  p->y = y;
        p->prev = p->next = nullptr;
        return p;
    }

    // ── Contour query ────────────────────────────────────────────────────
    // Scan forward from `hint` (hint->x <= x1) to collect:
    //   maxY    : maximum height in [x1, x2)
    //   endNode : last node whose x < x2  (endNode->next->x >= x2)
    //
    // Because `hint` is passed directly from the DFS parent, no binary
    // search is needed — we only walk the k segments inside [x1, x2).
    pair<int, ContourNode*> scanContour(int x1, int x2, ContourNode* hint) {
        int maxY = 0;
        ContourNode* cur = hint;
        for (;;) {
            if (cur->y > maxY) maxY = cur->y;
            if (!cur->next || cur->next->x >= x2) break;
            cur = cur->next;
        }
        return {maxY, cur};
    }

    // ── Contour update ───────────────────────────────────────────────────
    // Replace the contour over [x1, x2) with height newY.
    //   hint    : node with hint->x <= x1  (covers x1 before the update)
    //   endNode : last node with x < x2    (returned by scanContour)
    //
    // Returns: (newNode, nextNode)
    //   newNode  — represents [x1, ...) after update → hint for right child
    //   nextNode — represents [x2, ...) after update → hint for left  child
    pair<ContourNode*, ContourNode*> updateContour(
            int x1, int x2, int newY,
            ContourNode* hint, ContourNode* endNode) {

        // Step 1 — right boundary: ensure a node exists at x2.
        ContourNode* nextNode;
        if (endNode->next && endNode->next->x == x2) {
            nextNode = endNode->next;
        } else {
            // endNode->next->x > x2: split, preserving endNode's height.
            ContourNode* splitR = allocNode(x2, endNode->y);
            splitR->next = endNode->next;
            if (endNode->next) endNode->next->prev = splitR;
            endNode->next = splitR;
            splitR->prev  = endNode;
            nextNode = splitR;
        }

        // Step 2 — left boundary: ensure a node exists at x1.
        ContourNode* newNode;
        if (hint->x == x1) {
            newNode    = hint;        // reuse existing node
            newNode->y = newY;
        } else {
            // hint->x < x1: insert a new node between hint and hint->next.
            newNode       = allocNode(x1, newY);
            newNode->prev = hint;
            newNode->next = hint->next;
            if (hint->next) hint->next->prev = newNode;
            hint->next    = newNode;
        }

        // Step 3 — splice out all interior nodes in (x1, x2).
        // Abandoned nodes remain in the pool but are no longer reachable.
        if (newNode->next != nextNode) {
            newNode->next  = nextNode;
            nextNode->prev = newNode;
        }

        return {newNode, nextNode};
    }

    // ── DFS packing ──────────────────────────────────────────────────────
    // `hint` is the contour node covering the current x position.
    void dfs(vector<Block*>& blocks, int nodeIdx, int x,
             ContourNode* hint, int& totalW, int& totalH) {
        int bIdx = _nodes[nodeIdx].blockIdx;
        Block* blk = blocks[bIdx];
        int w  = blk->getWidth();
        int h  = blk->getHeight();
        int x2 = x + w;

        auto [y, endNode] = scanContour(x, x2, hint);
        blk->setPos(x, y, x2, y + h);

        auto [newNode, nextNode] = updateContour(x, x2, y + h, hint, endNode);

        if (x2    > totalW) totalW = x2;
        if (y + h > totalH) totalH = y + h;

        // Left child  → placed at x2; its contour hint is nextNode.
        if (_nodes[nodeIdx].left  >= 0)
            dfs(blocks, _nodes[nodeIdx].left,  x2, nextNode, totalW, totalH);
        // Right child → placed at x;  its contour hint is newNode.
        if (_nodes[nodeIdx].right >= 0)
            dfs(blocks, _nodes[nodeIdx].right, x,  newNode,  totalW, totalH);
    }

    // ── Tree topology helpers ────────────────────────────────────────────
    void deleteNode(int idx) {
        int p = _nodes[idx].parent;
        int l = _nodes[idx].left;
        int r = _nodes[idx].right;

        int child;
        if      (l == -1 && r == -1) { child = -1; }
        else if (l == -1)             { child =  r; }
        else if (r == -1)             { child =  l; }
        else {
            // Both children: hang r off the rightmost node of l's subtree
            child = l;
            int rm = l;
            while (_nodes[rm].right != -1) rm = _nodes[rm].right;
            _nodes[rm].right = r;
            _nodes[r].parent = rm;
        }

        if (child != -1) _nodes[child].parent = p;

        if (p == -1) { _root = child; }
        else {
            if (_nodes[p].left == idx) _nodes[p].left  = child;
            else                       _nodes[p].right = child;
        }

        _nodes[idx].parent = _nodes[idx].left = _nodes[idx].right = -1;
    }

    void insertNode(int idx, int target, bool toLeft) {
        if (target == -1) {
            _nodes[idx].left = _root;
            if (_root != -1) _nodes[_root].parent = idx;
            _root = idx;
            _nodes[idx].parent = -1;
            return;
        }
        if (toLeft) {
            int old = _nodes[target].left;
            _nodes[target].left  = idx;
            _nodes[idx].parent   = target;
            _nodes[idx].left     = old;
            if (old != -1) _nodes[old].parent = idx;
        } else {
            int old = _nodes[target].right;
            _nodes[target].right = idx;
            _nodes[idx].parent   = target;
            _nodes[idx].right    = old;
            if (old != -1) _nodes[old].parent = idx;
        }
    }
};

#endif
