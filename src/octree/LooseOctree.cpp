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
OctreeNode::OctreeNode(LooseOctree* const tree, OctreeNode* const pParent, const Vec3r& nodeCenter,
                       const Float halfWidth, const std::size_t depth) :
    _pTree(tree),
    _pParent(pParent),
    _center(nodeCenter),
    _lowerBound(nodeCenter - Vec3r(halfWidth, halfWidth, halfWidth)),
    _upperBound(nodeCenter + Vec3r(halfWidth, halfWidth, halfWidth)),
    _lowerExtendedBound(nodeCenter - 2.0 * Vec3r(halfWidth, halfWidth, halfWidth)),
    _upperExtendedBound(nodeCenter + 2.0 * Vec3r(halfWidth, halfWidth, halfWidth)),
    _halfWidth(halfWidth),
    _depth(depth),
    _maxDepth(tree->_maxDepth),
    _bIsLeaf(true) {
    // Must initialize primitive linked lists and counters
    for(int type = 0; type < OctreePrimitiveType::NumPrimitiveTypes; ++type) {
        clearPointData(static_cast<OctreePrimitiveType>(type));
    }
}

OctreeNode*
OctreeNode::getChildNode(const std::size_t childIdx) const {
#if defined(DEBUG) || defined(_DEBUG) || !defined(NDEBUG)
    LOG_IF(FATAL, (!m_pChildren)) << "Children node block is nullptr";
#endif
    return &_pChildren->m_Nodes[childIdx];
}

void
OctreeNode::clearPointData() {
    _pPointListHead[type] = nullptr;
    _pointCount[type]     = 0;

    if(!isLeaf()) {
        for(std::size_t childIdx = 0; childIdx < 8u; ++childIdx) {
            _pChildren->m_Nodes[childIdx].clearPointData(type);
        }
    }
}

void
OctreeNode::split() {
    if(!isLeaf() || _depth == _maxDepth) {
        return;
    }

    m_NodeSplitingLock.lock();
    if(isLeaf()) {
        //--------------------------------------------------------
        //
        //           6-------7
        //          /|      /|
        //         2-+-----3 |
        //         | |     | |   y
        //         | 4-----+-5   | z
        //         |/      |/    |/
        //         0-------1     +--x
        //
        //         0   =>   0, 0, 0
        //         1   =>   0, 0, 1
        //         2   =>   0, 1, 0
        //         3   =>   0, 1, 1
        //         4   =>   1, 0, 0
        //         5   =>   1, 0, 1
        //         6   =>   1, 1, 0
        //         7   =>   1, 1, 1
        //
        //--------------------------------------------------------

        _pChildren = _pTree->requestChildrenFromPool();

        const auto childHalfWidth = _halfWidth * static_cast<Float>(0.5);
        for(std::size_t childIdx = 0; childIdx < 8u; ++childIdx) {
            auto newCenter = _center;
            newCenter[0] += (childIdx & 1) ? childHalfWidth : -childHalfWidth;
            newCenter[1] += (childIdx & 2) ? childHalfWidth : -childHalfWidth;
            newCenter[2] += (childIdx & 4) ? childHalfWidth : -childHalfWidth;

            OctreeNode* const pChildNode = &_pChildren->m_Nodes[childIdx];

            // Placement new: re-use existing memory block, just call constructor to re-initialize data
            new(pChildNode) OctreeNode(_pTree, this, newCenter, childHalfWidth, _depth + 1u);
        }

        // Must explicitly mark as non-leaf node, and must do this after all children nodes are ready
        _bIsLeaf = false;
    }
    m_NodeSplitingLock.unlock();
}

void
OctreeNode::removeAllDescendants() {
    if(isLeaf()) {
        return;
    }

    // Must explicitly mark as leaf node
    _bIsLeaf = true;

    for(std::size_t childIdx = 0; childIdx < 8u; ++childIdx) {
        auto& pChildNode = _pChildren->m_Nodes[childIdx];
        pChildNode.removeAllDescendants();
    }
    _pTree->returnChildrenToPool(_pChildren);
}

void
OctreeNode::removeEmptyDescendants() {
    if(isLeaf()) {
        return;
    }

    bool bAllEmpty  = true;
    bool bAllLeaves = true;
    for(std::size_t childIdx = 0; childIdx < 8u; ++childIdx) {
        auto& pChildNode = _pChildren->m_Nodes[childIdx];
        pChildNode.removeEmptyDescendants();
        bAllLeaves &= pChildNode.isLeaf();

        for(int i = 0; i < OctreePrimitiveType::NumPrimitiveTypes; ++i) {
            bAllEmpty &= (pChildNode._pointCount[i] == 0);
        }
    }

    // Remove all 8 children nodes iff they are all leaf nodes and all empty nodes
    if(bAllEmpty && bAllLeaves) {
        _pTree->returnChildrenToPool(_pChildren);
        _bIsLeaf = true;
    }
}

void
OctreeNode::keepPoint(OctreePoint* const pPrimitive) {
    pPrimitive->m_pNode  = this;
    pPrimitive->m_bValid = true;

    m_PrimitiveLock[type].lock();
    pPrimitive->m_pNext   = _pPointListHead[type];
    _pPointListHead[type] = pPrimitive;
    _pointCount[type]    += 1u;
    m_PrimitiveLock[type].unlock();
}

void
OctreeNode::insertPoint(OctreePoint* const pPrimitive) {
    // Type alias, to reduce copy/past errors
    static const auto type = OctreePrimitiveType::Point;

    if(_depth == _maxDepth) {
        keepPoint(pPrimitive, type);
        return;
    }

    // Split node if this is a leaf node
    split();

    // Compute the index of the child node that contains this point
    std::size_t childIdx = 0;
    for(std::size_t dim = 0; dim < 3; ++dim) {
        if(_center[dim] < pPrimitive->m_Position[dim]) {
            childIdx |= (1 << dim);
        }
    }

    _pChildren->m_Nodes[childIdx].insertPoint(pPrimitive);
}

bool
OctreeNode::updateDebugGeometry() {
    if(_depth > _pTree->_maxLevelBoundaryLineGeneration) {
        return false;
    }

    int   renderCount = 0;
    Vec3r vertices[8];
    bool  rendered[8] { false, false, false, false,
                        false, false, false, false };

    // Also call add lines recursively
    for(std::size_t i = 0; i < 8; i++) {
        vertices[i]     = _center;
        vertices[i][0] += (i & 1) ? _halfWidth : -_halfWidth;
        vertices[i][1] += (i & 2) ? _halfWidth : -_halfWidth;
        vertices[i][2] += (i & 4) ? _halfWidth : -_halfWidth;

        if(!isLeaf()) {
            rendered[i] = _pChildren->m_Nodes[i].updateDebugGeometry();
            if(rendered[i]) {
                ++renderCount;
            }
        }
    }

    //--------------------------------------------------------
    //
    //           6-------7
    //          /|      /|
    //         2-+-----3 |
    //         | |     | |   y
    //         | 4-----+-5   | z
    //         |/      |/    |/
    //         0-------1     +--x
    //
    //         0   =>   0, 0, 0
    //         1   =>   0, 0, 1
    //         2   =>   0, 1, 0
    //         3   =>   0, 1, 1
    //         4   =>   1, 0, 0
    //         5   =>   1, 0, 1
    //         6   =>   1, 1, 0
    //         7   =>   1, 1, 1
    //
    //--------------------------------------------------------

    // No primitive in this node
    if(_pointCount[OctreePrimitiveType::Point] == 0
       && _pointCount[OctreePrimitiveType::Triangle] == 0
       && _pointCount[OctreePrimitiveType::AnalyticalGeometry] == 0) {
        if(!_pTree->_bBoundaryLineForNonEmptyParent) {
            return (renderCount > 0);
        }

        if(renderCount == 0                 // Children did not render
           && _pTree->_pRootNode != this) { // Not root node, and no data in this node)
            return false;
        }
    }

    if(renderCount < 8) { // If renderCount == 8 then no need to render this node
        const auto& debugLines = _pTree->m_DebugGeometry;
        for(int i = 0; i < 8; ++i) {
            if((i & 1) && (!rendered[i] || !rendered[i - 1])) {
                debugLines->appendVertex(vertices[i]);
                debugLines->appendVertex(vertices[i - 1]);
            }
            if((i & 2) && (!rendered[i] || !rendered[i - 2])) {
                debugLines->appendVertex(vertices[i]);
                debugLines->appendVertex(vertices[i - 2]);
            }
            if((i & 4) && (!rendered[i] || !rendered[i - 4])) {
                debugLines->appendVertex(vertices[i]);
                debugLines->appendVertex(vertices[i - 4]);
            }
        }
    }
    return true;
}

LooseOctree::LooseOctree(const Vec3r& center, const Float width, const Float minWidth,
                         const Float minWidthRatio /*= 1.0*/, const std::string name /*= "Octree"*/) :
    m_Name(name),
    _center(center),
    _width(width),
    m_MinWidthRatio(minWidthRatio),
    _minWidth(minWidth),
    _pRootNode(new OctreeNode(this, nullptr, center, width * static_cast<Float>(0.5), 1u)),
    _numAllocatedNodes(1u) {}

LooseOctree::~LooseOctree() {
    // Firstly clear data recursively
    clear();

    // Deallocate memory pool
    deallocateMemoryPool();

    // Remove root node
    delete _pRootNode;
}

void
LooseOctree::clear() {
    // Return all tree nodes to memory pool except the root node
    _pRootNode->removeAllDescendants();

    for(int type = 0; type < OctreePrimitiveType::NumPrimitiveTypes; ++type) {
        clearPoints(static_cast<OctreePrimitiveType>(type));
    }
    // Remove all geometry pointers
    m_sGeometryIndices.clear();

    // Set state to imcomplete
    _bCompleteBuild = false;
}

void
LooseOctree::clearPoints(const OctreePrimitiveType type) {
    // Recursively clear primitive data
    _pRootNode->clearPointData(type);

    // Remove primitives from tree
    if(_octreePoints[type].size() > 0) {
        for(const auto& pPrimitive: _octreePoints[type]) {
            removeGeometry(pPrimitive->m_GeomIdx);
        }
        _octreePoints[type].resize(0);
    }

    // Deallocate primitive memory blocks
    for(const auto pPrimitiveBlock: _pOctreePointBlocks[type]) {
        delete[] pPrimitiveBlock;
    }
    _pOctreePointBlocks[type].resize(0);
}

std::size_t
LooseOctree::getMaxNumPointInNodes() const {
    return tbb::parallel_reduce(_sActiveTreeNodeBlocks.range(),
                                0u,
                                [&](decltype(_sActiveTreeNodeBlocks)::const_range_type& r, std::size_t prevResult) -> std::size_t {
                                    for(auto it = r.begin(), iEnd = r.end(); it != iEnd; ++it) {
                                        const auto pNodeBlock = *it;
                                        for(std::size_t childIdx = 0; childIdx < 8u; ++childIdx) {
                                            const auto& pNode = pNodeBlock->m_Nodes[childIdx];
                                            for(int type = 0; type < OctreePrimitiveType::NumPrimitiveTypes; ++type) {
                                                if(prevResult < pNode.m_PrimitiveCounts[type]) {
                                                    prevResult = pNode.m_PrimitiveCounts[type];
                                                }
                                            }
                                        }
                                    }
                                    return prevResult;
                                },
                                [](const std::size_t x, const std::size_t y) -> std::size_t {
                                    return x > y ? x : y;
                                });
}

std::size_t
Octree::addPointSet(const std::shared_ptr<PointSet>& pointset) {
    // Type alias, to reduce copy/past errors
    static const auto type = static_cast<int>(OctreePrimitiveType::Point);

    const auto pGeometry = static_cast<Geometry*>(pointset.get());
    const auto geomIdx   = pGeometry->getGlobalIndex();
    addGeometry(geomIdx);

    const auto numNewPrimitives = static_cast<std::size_t>(pointset->getNumVertices());
    const auto pPrimitiveBlock  = new OctreePrimitive[numNewPrimitives];
    m_pPrimitiveBlocks[type].push_back(pPrimitiveBlock);

    auto& vPrimitivePtrs = m_vPrimitivePtrs[type];
    vPrimitivePtrs.reserve(vPrimitivePtrs.size() + numNewPrimitives);
    for(std::size_t idx = 0; idx < numNewPrimitives; ++idx) {
        const auto pPrimitive = &pPrimitiveBlock[idx];
        new(pPrimitive) OctreePrimitive(pGeometry, geomIdx, idx); // Placement new
        vPrimitivePtrs.push_back(pPrimitive);
    }

    LOG(INFO) << "Added " << numNewPrimitives << " points to " << m_Name;
    return numNewPrimitives;
}

void
Octree::build() {
    if(m_sGeometryIndices.size() == 0) {
        LOG(WARNING) << "There was not any geometry added in the tree named '" << m_Name << "'";
        return;
    }

    // Compute the minimum bounding box of non-point primitives
    if(m_vPrimitivePtrs[OctreePrimitiveType::Point].size() == 0
       && (m_vPrimitivePtrs[OctreePrimitiveType::Triangle].size() > 0
           || m_vPrimitivePtrs[OctreePrimitiveType::AnalyticalGeometry].size() > 0)) {
        Float minWidth = MAX_Float;
        for(int type = OctreePrimitiveType::Triangle; type <= OctreePrimitiveType::AnalyticalGeometry; ++type) {
            const auto& vPrimitivePtrs = m_vPrimitivePtrs[type];
            if(vPrimitivePtrs.size() == 0) {
                continue;
            }
            const auto primitiveMinWidth = tbb::parallel_reduce(tbb::blocked_range<size_t>(0, vPrimitivePtrs.size()),
                                                                MAX_Float,
                                                                [&](const tbb::blocked_range<size_t>& r, Float prevResult) -> Float {
                                                                    for(auto i = r.begin(), iEnd = r.end(); i != iEnd; ++i) {
                                                                        const auto pPrimitive = vPrimitivePtrs[i];
                                                                        computePrimitiveBoundingBox(pPrimitive, static_cast<OctreePrimitiveType>(type));

                                                                        Vec3r widths;
                                                                        for(std::size_t dim = 0; dim < 3; ++dim) {
                                                                            widths[dim] = pPrimitive->m_UpperCorner[dim] - pPrimitive->m_LowerCorner[dim];
                                                                        }
                                                                        auto maxBoxWidth = widths[0];
                                                                        maxBoxWidth      = maxBoxWidth < widths[1] ? widths[1] : maxBoxWidth;
                                                                        maxBoxWidth      = maxBoxWidth < widths[2] ? widths[2] : maxBoxWidth;
                                                                        prevResult       = prevResult > maxBoxWidth ? maxBoxWidth : prevResult;
                                                                    }
                                                                    return prevResult;
                                                                },
                                                                [](const Float x, const Float y) -> Float {
                                                                    return x < y ? x : y;
                                                                });

            minWidth = minWidth < primitiveMinWidth ? minWidth : primitiveMinWidth;
        }

        if(minWidth < 1e-8) {
            LOG(WARNING) << "Object/triangles have too small size";
        } else {
            m_MinWidth = m_MinWidthRatio * minWidth;
        }
    }

    // Compute max depth that the tree can reach
    m_MaxDepth = 1u;
    std::size_t numLevelNodes   = 1u;
    std::size_t maxNumTreeNodes = 1u;
    Float       nodeWidth       = m_Width;

    while(nodeWidth * static_cast<Float>(0.5) > m_MinWidth) {
        ++m_MaxDepth;
        numLevelNodes   *= 8u;
        maxNumTreeNodes += numLevelNodes;
        nodeWidth       *= static_cast<Float>(0.5);
    }
    m_pRootNode->m_MaxDepth = m_MaxDepth;
    rebuild();
    m_bCompleteBuild = true;

    LOG(INFO) << m_Name << " generated, center = [" << m_Center[0] << ", " << m_Center[1] << ", " << m_Center[2]
              << "], width = " << m_Width << ", min width = " << m_MinWidth
              << ", max depth = " << m_MaxDepth << ", max num. nodes = " << maxNumTreeNodes;
}

void
Octree::update() {
    if(!m_bCompleteBuild) {
        build();
    }
    (!m_bAlwaysRebuild) ? incrementalUpdate() : rebuild();
}

void
Octree::rebuild() {
    // Recursively remove all tree nodes other than root node
    m_pRootNode->removeAllDescendants();

    // Clear root node data
    for(int type = 0; type < OctreePrimitiveType::NumPrimitiveTypes; ++type) {
        m_pRootNode->clearPrimitiveData(static_cast<OctreePrimitiveType>(type));
    }

    // Populate all primitives to tree nodes in a top-down manner
    populatePointPrimitives();
}

void
Octree::populatePointPrimitives() {
    const auto& vPrimitivePtrs = m_vPrimitivePtrs[OctreePrimitiveType::Point];
    if(vPrimitivePtrs.size() == 0) {
        return;
    }
    ParallelUtils::parallelFor(vPrimitivePtrs.size(),
                               [&](const size_t idx) {
                                   const auto pPrimitive  = vPrimitivePtrs[idx];
                                   const auto pointset    = static_cast<PointSet*>(pPrimitive->m_pGeometry);
                                   const auto point       = pointset->getVertexPosition(pPrimitive->m_Idx);
                                   pPrimitive->m_Position = { point[0], point[1], point[2] };
                                   m_pRootNode->insertPoint(pPrimitive);
                               });
}

void
Octree::incrementalUpdate() {
    // For all primitives, update their positions (if point) or bounding box (if non-point)
    // Then, check their validity (valid primitive = it is still loosely contained in the node's bounding box)
    updatePositionAndCheckValidity();
    updateBoundingBoxAndCheckValidity(OctreePrimitiveType::Triangle);
    updateBoundingBoxAndCheckValidity(OctreePrimitiveType::AnalyticalGeometry);

    // Remove all invalid primitives from tree nodes
    removeInvalidPrimitivesFromNodes();

    // Insert the invalid primitives back to the tree
    reinsertInvalidPrimitives(OctreePrimitiveType::Point);
    reinsertInvalidPrimitives(OctreePrimitiveType::Triangle);
    reinsertInvalidPrimitives(OctreePrimitiveType::AnalyticalGeometry);

    // Recursively remove all empty nodes, returning them to memory pool for recycling
    m_pRootNode->removeEmptyDescendants();
}

void
Octree::updatePositionAndCheckValidity() {
    const auto& vPrimitivePtrs = m_vPrimitivePtrs[OctreePrimitiveType::Point];
    if(vPrimitivePtrs.size() == 0) {
        return;
    }
    ParallelUtils::parallelFor(vPrimitivePtrs.size(),
                               [&](const size_t idx) {
                                   const auto pPrimitive = vPrimitivePtrs[idx];
                                   const auto pointset   = static_cast<PointSet*>(pPrimitive->m_pGeometry);
                                   const auto point      = pointset->getVertexPosition(pPrimitive->m_Idx);

                                   // Cache the position
                                   pPrimitive->m_Position = { point[0], point[1], point[2] };

                                   auto pNode = pPrimitive->m_pNode;
                                   if(!pNode->looselyContains(point) && pNode != m_pRootNode) {
                                       // Go up, find the node tightly containing it (or stop if reached root node)
                                       while(pNode != m_pRootNode) {
                                           pNode = pNode->m_pParent; // Go up one level
                                           if(pNode->contains(point) || pNode == m_pRootNode) {
                                               pPrimitive->m_bValid = false;
                                               pPrimitive->m_pNode  = pNode;
                                               break;
                                           }
                                       }
                                   } else {
                                       pPrimitive->m_bValid = (pNode != m_pRootNode) ? true : false;
                                   }
                               });
}

void
Octree::removeInvalidPrimitivesFromNodes() {
    if(m_sActiveTreeNodeBlocks.size() == 0) {
        return;
    }
    tbb::parallel_for(m_sActiveTreeNodeBlocks.range(),
                      [&](decltype(m_sActiveTreeNodeBlocks)::const_range_type& r) {
                          for(auto it = r.begin(), iEnd = r.end(); it != iEnd; ++it) {
                              const auto pNodeBlock = *it;
                              for(std::size_t childIdx = 0; childIdx < 8u; ++childIdx) {
                                  auto& pNode = pNodeBlock->m_Nodes[childIdx];
                                  for(int type = 0; type < OctreePrimitiveType::NumPrimitiveTypes; ++type) {
                                      const auto pOldHead = pNode.m_pPrimitiveListHeads[type];
                                      if(!pOldHead) {
                                          continue;
                                      }

                                      OctreePrimitive* pIter    = pOldHead;
                                      OctreePrimitive* pNewHead = nullptr;
                                      std::size_t count         = 0;
                                      while(pIter) {
                                          const auto pNext = pIter->m_pNext;
                                          if(pIter->m_bValid) {
                                              pIter->m_pNext = pNewHead;
                                              pNewHead       = pIter;
                                              ++count;
                                          }
                                          pIter = pNext;
                                      }
                                      pNode.m_pPrimitiveListHeads[type] = pNewHead;
                                      pNode.m_PrimitiveCounts[type]     = count;
                                  }
                              }
                          }
                      });
}

void
Octree::reinsertInvalidPrimitives(const OctreePrimitiveType type) {
    const auto& vPrimitivePtrs = m_vPrimitivePtrs[type];
    if(vPrimitivePtrs.size() == 0) {
        return;
    }
    ParallelUtils::parallelFor(vPrimitivePtrs.size(),
                               [&](const size_t idx) {
                                   const auto pPrimitive = vPrimitivePtrs[idx];
                                   if(pPrimitive->m_bValid) {
                                       return;
                                   }

                                   const auto pNode = pPrimitive->m_pNode;
                                   (type == OctreePrimitiveType::Point) ? pNode->insertPoint(pPrimitive) : pNode->insertNonPointPrimitive(pPrimitive, type);
                               });
}

OctreeNodeBlock*
Octree::requestChildrenFromPool() {
    m_PoolLock.lock();
    if(m_NumAvaiableBlocksInPool == 0) {
        // Allocate 64 more node blocks and put to the pool
        allocateMoreNodeBlock(64u);
    }

    const auto pNodeBlock = m_pNodeBlockPoolHead;
    m_pNodeBlockPoolHead       = pNodeBlock->m_NextBlock;
    m_NumAvaiableBlocksInPool -= 1u;
    m_PoolLock.unlock();
    m_sActiveTreeNodeBlocks.insert(pNodeBlock);
    return pNodeBlock;
}

void
Octree::returnChildrenToPool(OctreeNodeBlock* const pNodeBlock) {
    m_PoolLock.lock();
    pNodeBlock->m_NextBlock    = m_pNodeBlockPoolHead;
    m_pNodeBlockPoolHead       = pNodeBlock;
    m_NumAvaiableBlocksInPool += 1u;
    m_sActiveTreeNodeBlocks.unsafe_erase(pNodeBlock);
    m_PoolLock.unlock();
}

void
Octree::allocateMoreNodeBlock(const std::size_t numBlocks) {
    // This is not a thead-safe function, thus it should be called only from a thread-safe function
    // And it must be called only from the requestChildrenFromPool() function
    const auto pBigBlock = new OctreeNodeBlock[numBlocks];
    m_pNodeBigBlocks.push_back(pBigBlock);
    for(std::size_t i = 0; i < numBlocks; ++i) {
        const auto pBlock = &pBigBlock[i];
        pBlock->m_NextBlock  = m_pNodeBlockPoolHead;
        m_pNodeBlockPoolHead = pBlock;
    }
    m_NumAvaiableBlocksInPool += numBlocks;
    m_NumAllocatedNodes       += numBlocks * 8u;
}

void
Octree::deallocateMemoryPool() {
    LOG_IF(FATAL, (m_NumAllocatedNodes != m_NumAvaiableBlocksInPool * 8u + 1u))
        << "Internal data corrupted, may be all nodes were not returned from tree";

    for(const auto pBigBlock: m_pNodeBigBlocks) {
        delete[] pBigBlock;
    }
    m_pNodeBigBlocks.resize(0);
    m_NumAllocatedNodes       = 1u; // root node still remains
    m_NumAvaiableBlocksInPool = 0u;
}

std::shared_ptr<DebugRenderGeometry>
Octree::getDebugGeometry(const std::size_t maxLevel, bool bDrawNonEmptyParent /*= true*/) {
    m_MaxLevelDebugRender = maxLevel;
    m_bDrawNonEmptyParent = bDrawNonEmptyParent;
    if(!m_DebugGeometry) {
        m_DebugGeometry.reset();
    }

    // Create debug geometry and set default rendering mateirial
    m_DebugGeometry = std::make_shared<DebugRenderLines>("OctreeDebugRendering");

    // Update debug rendering data (if any)
    m_pRootNode->updateDebugGeometry();
    m_DebugGeometry->setDataModified(true);

    return std::static_pointer_cast<DebugRenderGeometry>(m_DebugGeometry);
}

void
Octree::updateDebugGeometry() {
    LOG_IF(FATAL, (!m_DebugGeometry)) << "Debug geometry has not been created";
    m_DebugGeometry->clear();
    m_pRootNode->updateDebugGeometry();
    m_DebugGeometry->setDataModified(true);
}
} }
