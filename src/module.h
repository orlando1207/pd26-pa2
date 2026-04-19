#ifndef MODULE_H
#define MODULE_H

#include <vector>
#include <string>
#include <climits>
#include <algorithm>
using namespace std;

class Terminal
{
public:
    Terminal(const string& name, int x, int y) :
        _name(name), _x1(x), _y1(y), _x2(x), _y2(y) { }
    virtual ~Terminal() { }

    const string& getName() const { return _name; }
    int getX1() const { return _x1; }
    int getY1() const { return _y1; }
    int getX2() const { return _x2; }
    int getY2() const { return _y2; }

    void setPos(int x1, int y1, int x2, int y2) {
        _x1 = x1; _y1 = y1; _x2 = x2; _y2 = y2;
    }

protected:
    string _name;
    int _x1, _y1, _x2, _y2;
};

class Block : public Terminal
{
public:
    Block(const string& name, int w, int h) :
        Terminal(name, 0, 0), _w(w), _h(h), _rotate(false) { }
    ~Block() { }

    int getWidth() const { return _rotate ? _h : _w; }
    int getHeight() const { return _rotate ? _w : _h; }
    int getArea() const { return _w * _h; }
    bool getRotate() const { return _rotate; }
    void setRotate(bool r) { _rotate = r; }

private:
    int _w, _h;
    bool _rotate;
};

class Net
{
public:
    Net() { }
    ~Net() { }

    const vector<Terminal*>& getTermList() const { return _termList; }
    void addTerm(Terminal* t) { _termList.push_back(t); }

    double calcHPWL() const {
        int minX = INT_MAX, minY = INT_MAX, maxX = INT_MIN, maxY = INT_MIN;
        for (auto* t : _termList) {
            int cx = (t->getX1() + t->getX2()) / 2;
            int cy = (t->getY1() + t->getY2()) / 2;
            minX = min(minX, cx); maxX = max(maxX, cx);
            minY = min(minY, cy); maxY = max(maxY, cy);
        }
        return (maxX - minX) + (maxY - minY);
    }

private:
    vector<Terminal*> _termList;
};

#endif
