#pragma once
#include <iostream>
#include <vector>
#include "wx/vector.h"
#include "wx/gdicmn.h"

class PlotElementsBuffer
{
public:
	explicit PlotElementsBuffer() : size(100), buffer(100, {0,0}), head(0), count(0) {}
    explicit PlotElementsBuffer(const uint64_t size) : 
        size(size), buffer(size, { 0,0 }), head(0), count(0) {}

    void AddElement(const wxRealPoint& element) {
        buffer.push_back(element);
        if (buffer.size() > size) {
            buffer.erase(buffer.begin());
        }
    }

    void Clear()
    {
        buffer.clear();
    }

    wxVector <wxRealPoint> getBuffer() const { return buffer; }

private:
    void IncrementIndex(size_t& index) const {
        index = (index + 1) % size;
    }

    size_t size;
    wxVector <wxRealPoint> buffer;
    size_t head;
    size_t count;
};

