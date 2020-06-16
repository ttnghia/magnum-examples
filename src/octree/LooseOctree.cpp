/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vladimír Vondruš <mosra@centrum.cz>
        2020 — Nghia Truong <nghiatruong.vn@gmail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "LooseOctree.h"

namespace Magnum { namespace Examples {
OctreeNode::OctreeNode(LooseOctree* const tree, OctreeNode* const pParent,
                       const Vector3& nodeCenter, const Float halfWidth,
                       const size_t depth) :
    _pTree(tree),
    _pParent(pParent),
    _pChildren(nullptr),
    _center(nodeCenter),
    _lowerBound(nodeCenter - Vector3(halfWidth, halfWidth, halfWidth)),
    _upperBound(nodeCenter + Vector3(halfWidth, halfWidth, halfWidth)),
    _lowerBoundExtended(nodeCenter - 2 * Vector3(halfWidth, halfWidth, halfWidth)),
    _upperBoundExtended(nodeCenter + 2 * Vector3(halfWidth, halfWidth, halfWidth)),
    _halfWidth(halfWidth),
    _depth(depth),
    _maxDepth(tree->_maxDepth),
    _bIsLeaf(true) {}

OctreeNode* OctreeNode::getChildNode(const std::size_t childIdx) const {
    CORRADE_INTERNAL_ASSERT(_pChildren != nullptr);
    return &_pChildren->_nodes[childIdx];
}

void OctreeNode::clearPointData() {
    _pPointListHead = nullptr;
    _pointCount     = 0;
    if(!isLeaf()) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            _pChildren->_nodes[childIdx].clearPointData();
        }
    }
}

void OctreeNode::split() {
    if(!isLeaf() || _depth == _maxDepth) {
        return;
    }

    if(isLeaf()) {
        /*--------------------------------------------------------
         *
         *           6-------7
         *          /|      /|
         *         2-+-----3 |
         *         | |     | |   y
         *         | 4-----+-5   | z
         *         |/      |/    |/
         *         0-------1     +--x
         *
         *         0   =>   0, 0, 0
         *         1   =>   0, 0, 1
         *         2   =>   0, 1, 0
         *         3   =>   0, 1, 1
         *         4   =>   1, 0, 0
         *         5   =>   1, 0, 1
         *         6   =>   1, 1, 0
         *         7   =>   1, 1, 1
         *
         *--------------------------------------------------------*/

        _pChildren = _pTree->requestChildrenFromPool();

        const auto childHalfWidth = _halfWidth * static_cast<Float>(0.5);
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            auto newCenter = _center;
            newCenter[0] += (childIdx & 1) ? childHalfWidth : -childHalfWidth;
            newCenter[1] += (childIdx & 2) ? childHalfWidth : -childHalfWidth;
            newCenter[2] += (childIdx & 4) ? childHalfWidth : -childHalfWidth;

            OctreeNode* const pChildNode = &_pChildren->_nodes[childIdx];

            /* Placement new: re-use existing memory block, just call constructor to re-initialize data */
            new(pChildNode) OctreeNode(_pTree, this, newCenter, childHalfWidth, _depth + 1u);
        }

        /* Must explicitly mark as non-leaf node, and must do this after all children nodes are ready */
        _bIsLeaf = false;
    }
}

void OctreeNode::removeAllDescendants() {
    if(isLeaf()) {
        return;
    }

    /* Must explicitly mark as leaf node */
    _bIsLeaf = true;

    for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
        auto& pChildNode = _pChildren->_nodes[childIdx];
        pChildNode.removeAllDescendants();
    }
    _pTree->returnChildrenToPool(_pChildren);
}

void OctreeNode::removeEmptyDescendants() {
    if(isLeaf()) {
        return;
    }

    bool bAllEmpty  = true;
    bool bAllLeaves = true;
    for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
        auto& pChildNode = _pChildren->_nodes[childIdx];
        pChildNode.removeEmptyDescendants();
        bAllLeaves &= pChildNode.isLeaf();
        bAllEmpty  &= (pChildNode._pointCount == 0);
    }

    // Remove all 8 children nodes iff they are all leaf nodes and all empty nodes
    if(bAllEmpty && bAllLeaves) {
        _pTree->returnChildrenToPool(_pChildren);
        _bIsLeaf = true;
    }
}

void OctreeNode::keepPoint(OctreePoint* const pPoint) {
    pPoint->pNode   = this;
    pPoint->bValid  = true;
    pPoint->pNext   = _pPointListHead;
    _pPointListHead = pPoint;
    _pointCount    += 1u;
}

void OctreeNode::insertPoint(OctreePoint* const pPoint) {
    if(_depth == _maxDepth) {
        keepPoint(pPoint);
        return;
    }

    /* Split node if this is a leaf node */
    split();

    /* Compute the index of the child node that contains this point */
    const Vector3 ppos     = pPoint->position;
    std::size_t   childIdx = 0;
    for(std::size_t dim = 0; dim < 3; ++dim) {
        if(_center[dim] < ppos[dim]) {
            childIdx |= (1ull << dim);
        }
    }
    _pChildren->_nodes[childIdx].insertPoint(pPoint);
}

LooseOctree::~LooseOctree() {
    /* Firstly clear data recursively */
    clear();

    /* Deallocate memory pool */
    deallocateMemoryPool();

    /* Remove root node */
    delete _pRootNode;
}

void LooseOctree::clear() {
    /* Return all tree nodes to memory pool except the root node */
    _pRootNode->removeAllDescendants();

    clearPoints();

    /* Set state to imcomplete build */
    _bCompleteBuild = false;
}

void LooseOctree::clearPoints() {
    /* Recursively clear point data */
    _pRootNode->clearPointData();

    /* Deallocate memory blocks */
    if(_octreePointsPtr != nullptr && _nPoints > 0) {
        delete _octreePointsPtr;
        _nPoints = 0;
    }
}

std::size_t LooseOctree::getMaxNumPointInNodes() const {
    std::size_t count { 0 };
    for(const OctreeNodeBlock* pNodeBlock : _sActiveTreeNodeBlocks) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            const auto& pNode = pNodeBlock->_nodes[childIdx];
            if(count < pNode._pointCount) {
                count = pNode._pointCount;
            }
        }
    }
    return count;
}

void LooseOctree::setPoints(const std::vector<Vector3>& points) {
    clearPoints();
    _nPoints = points.size();
    if(_nPoints == 0) {
        return;
    }
    _octreePointsPtr = new OctreePoint[_nPoints];
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        const auto pPoint = &_octreePointsPtr[idx];
        new(pPoint) OctreePoint(points[idx], idx); /* placement new */
    }
}

void LooseOctree::updatePoints(const std::vector<Vector3>& points) {
    if(_nPoints != points.size()) {
        Fatal{} << "Invalid points array.";
    }
    if(_nPoints == 0) {
        return;
    }
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        _octreePointsPtr[idx].position = points[idx];
    }
}

void LooseOctree::build() {
    /* Compute max depth that the tree can reach */
    _maxDepth = 0u;
    std::size_t numLevelNodes   = 1u;
    std::size_t maxNumTreeNodes = 1u;
    Float       nodeHalfWidth   = _halfWidth;

    while(nodeHalfWidth > _minHalfWidth) {
        ++_maxDepth;
        numLevelNodes   *= 8;
        maxNumTreeNodes += numLevelNodes;
        nodeHalfWidth   *= static_cast<Float>(0.5);
    }
    _pRootNode->_maxDepth = _maxDepth;
    rebuild();
    _bCompleteBuild = true;

    Debug() << "Octree info:";
    Debug() << "       Center:" << _center;
    Debug() << "       Half width:" << _halfWidth;
    Debug() << "       Min half width:" << _minHalfWidth;
    Debug() << "       Max depth:" << _maxDepth;
    Debug() << "       Max tree nodes:" << maxNumTreeNodes;
}

void LooseOctree::update() {
    if(!_bCompleteBuild) {
        build();
    }
    (!_bAlwaysRebuild) ? incrementalUpdate() : rebuild();
}

void LooseOctree::rebuild() {
    /* Recursively remove all tree nodes other than root node */
    _pRootNode->removeAllDescendants();

    /* Clear root node point data */
    _pRootNode->clearPointData();

    /* Populate all points to tree nodes in a top-down manner */
    populatePoints();
}

void LooseOctree::populatePoints() {
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        _pRootNode->insertPoint(&_octreePointsPtr[idx]);
    }
}

void LooseOctree::incrementalUpdate() {
    checkValidity();
    removeInvalidPointsFromNodes();
    reinsertInvalidPoints();

    /* Recursively remove all empty nodes, returning them to memory pool for recycling */
    _pRootNode->removeEmptyDescendants();
}

void LooseOctree::checkValidity() {
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        OctreePoint&  point = _octreePointsPtr[idx];
        OctreeNode*   pNode = point.pNode;
        const Vector3 ppos  = point.position;
        if(!pNode->looselyContains(ppos) && pNode != _pRootNode) {
            /* Go up, find the node tightly containing it (or stop if reached root node) */
            while(pNode != _pRootNode) {
                pNode = pNode->_pParent;
                if(pNode->contains(ppos) || pNode == _pRootNode) {
                    point.bValid = false;
                    point.pNode  = pNode;
                    break;
                }
            }
        } else {
            point.bValid = (pNode != _pRootNode) ? true : false;
        }
    }
}

void LooseOctree::removeInvalidPointsFromNodes() {
    for(OctreeNodeBlock* const pNodeBlock: _sActiveTreeNodeBlocks) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            OctreeNode&        pNode    = pNodeBlock->_nodes[childIdx];
            OctreePoint* const pOldHead = pNode._pPointListHead;
            if(!pOldHead) {
                continue;
            }

            OctreePoint* pIter    = pOldHead;
            OctreePoint* pNewHead = nullptr;
            std::size_t  count    = 0;
            while(pIter) {
                const auto pNext = pIter->pNext;
                if(pIter->bValid) {
                    pIter->pNext = pNewHead;
                    pNewHead     = pIter;
                    ++count;
                }
                pIter = pNext;
            }
            pNode._pPointListHead = pNewHead;
            pNode._pointCount     = count;
        }
    }
}

void LooseOctree::reinsertInvalidPoints() {
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        const auto point = &_octreePointsPtr[idx];
        if(!point->bValid) {
            _pRootNode->insertPoint(point);
        }
    }
}

OctreeNodeBlock* LooseOctree::requestChildrenFromPool() {
    if(_numAvaiableBlocksInPool == 0) {
        /* Allocate 64 more node blocks and put to the pool */
        const auto pBigBlock = new OctreeNodeBlock[64];
        _pNodeBigBlocks.push_back(pBigBlock);
        for(std::size_t i = 0; i < 64; ++i) {
            const auto pBlock = &pBigBlock[i];
            pBlock->_nextBlock  = _pNodeBlockPoolHead;
            _pNodeBlockPoolHead = pBlock;
        }
        _numAvaiableBlocksInPool += 64;
        _numAllocatedNodes       += 64 * 8;
    }

    const auto pNodeBlock = _pNodeBlockPoolHead;
    _pNodeBlockPoolHead       = pNodeBlock->_nextBlock;
    _numAvaiableBlocksInPool -= 1u;
    _sActiveTreeNodeBlocks.insert(pNodeBlock);
    return pNodeBlock;
}

void LooseOctree::returnChildrenToPool(OctreeNodeBlock* const pNodeBlock) {
    pNodeBlock->_nextBlock    = _pNodeBlockPoolHead;
    _pNodeBlockPoolHead       = pNodeBlock;
    _numAvaiableBlocksInPool += 1u;
    _sActiveTreeNodeBlocks.erase(pNodeBlock);
}

void LooseOctree::deallocateMemoryPool() {
    if(_numAllocatedNodes != _numAvaiableBlocksInPool * 8 + 1u) {
        Fatal{} << "Internal data corrupted, may be all nodes were not returned from tree";
    }

    for(const auto pBigBlock: _pNodeBigBlocks) {
        delete[] pBigBlock;
    }
    _pNodeBigBlocks.resize(0);
    _numAllocatedNodes       = 1u; /* root node still remains */
    _numAvaiableBlocksInPool = 0u;
}
} }
