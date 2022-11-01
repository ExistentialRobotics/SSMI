#ifndef SEMANTIC_OCTOMAP_SEMANTICOCTREE_HXX
#define SEMANTIC_OCTOMAP_SEMANTICOCTREE_HXX

namespace octomap {

    // Tree implementation  --------------------------------------
    template<class SEMANTICS>
    SemanticOcTree<SEMANTICS>::SemanticOcTree(double resolution)
            : OccupancyOcTreeBase<SemanticOcTreeNode<SEMANTICS> >(this->resolution),
                    phiTree(), psiTree(), maxLogOddsTree(), minLogOddsTree(), minOccupancyLogOdds()
    {
        semanticOcTreeMemberInit.ensureLinking();
    };

    template<class SEMANTICS>
    bool SemanticOcTree<SEMANTICS>::pruneNode(SemanticOcTreeNode<SEMANTICS>* node) {
        // Same as ColorOcTree
        if (!isNodeCollapsible(node))
            return false;

        // Set value to children's values (all assumed equal)
        node->copyData(*(this->getNodeChild(node, 0)));

        if (node->isColorSet()) // TODO check
            node->setColor(node->getAverageChildColor());

        // Delete children
        for (unsigned int i=0;i<8;i++) {
            this->deleteNodeChild(node, i);
        }
        delete[] node->children;
        node->children = NULL;

        return true;
    }

    template<class SEMANTICS>
    bool SemanticOcTree<SEMANTICS>::isNodeCollapsible(const SemanticOcTreeNode<SEMANTICS>* node) const
    {
        // All children must exist, must not have children of
        // their own and have same occupancy probability and same log-odds vector
        if(!this->nodeChildExists(node, 0))
            return false;
        const SemanticOcTreeNode<SEMANTICS>* firstChild = this->getNodeChild(node, 0);
        if(this->nodeHasChildren(firstChild))
            return false;
        bool firstChildFree = firstChild->getValue() <= this->minOccupancyLogOdds;
        for (unsigned int i = 1; i<8; i++) {
            // Compare nodes using their occupancy and log-odds vector, ignoring color for pruning
            if (!this->nodeChildExists(node, i) || this->nodeHasChildren(this->getNodeChild(node, i))
                || !(this->getNodeChild(node, i)->getValue() == firstChild->getValue())
                || !(this->getNodeChild(node, i)->getSemantics() == firstChild->getSemantics() || firstChildFree))
                return false;
        }
        return true;
    }

    template<class SEMANTICS>
    void SemanticOcTree<SEMANTICS>::setUseSemanticColor(bool use)
    {
        // Traverse all tree nodes
        for(typename SemanticOcTree<SEMANTICS>::tree_iterator it = this->begin_tree(), end=this->end_tree(); it!= end; ++it)
            it->use_semantic_color = use;
    }

    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::setNodeColor(const OcTreeKey& key,
                                                                           uint8_t r,
                                                                           uint8_t g,
                                                                           uint8_t b)
    {
        SemanticOcTreeNode<SEMANTICS>* n = this->search (key);
        if (n != 0) {
            n->setColor(r, g, b);
        }
        return n;
    }

    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::averageNodeColor(SemanticOcTreeNode<SEMANTICS>* node, uint8_t r,
                                                                               uint8_t g, uint8_t b)
    {
        if (node != 0) {
            if (node->isColorSet()) {
                ColorOcTreeNode::Color prev_color = node->getColor();
                node->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
            }
            else {
                node->setColor(r, g, b);
            }
        }
        return node;
    }

    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::updateNodeSemantics(SemanticOcTreeNode<SEMANTICS>* node,
                                                                                  ColorOcTreeNode::Color obs)
    {
        if (node != 0)
        {
            
            SEMANTICS sem;
            
            if (node->isSemanticsSet())
            {
                sem = SEMANTICS::fuseObs(node->semantics, obs, phiTree, psiTree, maxLogOddsTree, minLogOddsTree);
            }
            else
            {
                sem = SEMANTICS::initSemantics(obs, node->getLogOdds(), phiTree, psiTree, maxLogOddsTree, minLogOddsTree);
            }
            
            float logOddsValue = SEMANTICS::getOccFromSem(sem);
            
            node->setSemantics(sem);
            node->setLogOdds(logOddsValue);
        }
        
        return node;
    }
    
    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::updateFreeNode(SemanticOcTreeNode<SEMANTICS>* node)
    {
        if(node->isSemanticsSet())
        {
            SEMANTICS sem = SEMANTICS::fuseObsFree(node->semantics, phiTree, minLogOddsTree);
            float logOddsValue = SEMANTICS::getOccFromSem(sem);
            
            node->setSemantics(sem);
            node->setLogOdds(logOddsValue);
        } else
        {
            float nodeLogOdds = node->getLogOdds() + phiTree;
            if(nodeLogOdds < minOccupancyLogOdds)
                nodeLogOdds = minOccupancyLogOdds;
            
            node->setLogOdds(nodeLogOdds);
        }
        
        return node;
    }

    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::updateNode(const OcTreeKey& key, bool occupied,
                                                                         const ColorOcTreeNode::Color& class_obs,
                                                                         const ColorOcTreeNode::Color& color_obs,
                                                                         bool lazy_eval)
    {
        // early abort (no change will happen).
        // may cause an overhead in some configuration, but more often helps
        SemanticOcTreeNode<SEMANTICS>* leaf = this->search(key);
        // no change: node already at threshold
        if (leaf && !checkNeedsUpdate(leaf, occupied, class_obs))
        {
          return leaf;
        }

        bool createdRoot = false;
        if (this->root == NULL){
          this->root = new SemanticOcTreeNode<SEMANTICS>();
          this->tree_size++;
          createdRoot = true;
        }

        return updateNodeRecurs(this->root, createdRoot, key, 0, occupied, class_obs, color_obs, lazy_eval);
    }

    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::updateNode(float x, float y, float z, bool occupied,
                                                                         const ColorOcTreeNode::Color& class_obs,
                                                                         const ColorOcTreeNode::Color& color_obs,
                                                                         bool lazy_eval)
    {
        OcTreeKey key;
        if (!this->coordToKeyChecked(x, y, z, key))
            return NULL;
        return updateNode(key, occupied, class_obs, color_obs, lazy_eval);
    }

    template<class SEMANTICS>
    SemanticOcTreeNode<SEMANTICS>* SemanticOcTree<SEMANTICS>::updateNodeRecurs(SemanticOcTreeNode<SEMANTICS>* node, bool node_just_created, const OcTreeKey& key,
                                                                               unsigned int depth, bool occupied, const ColorOcTreeNode::Color& class_obs,
                                                                               const ColorOcTreeNode::Color& color_obs, bool lazy_eval)
    {
        bool created_node = false;

        assert(node);

        // follow down to last level
        if (depth < this->tree_depth) {
          unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
          if (!this->nodeChildExists(node, pos)) {
            // child does not exist, but maybe it's a pruned node?
            if (!this->nodeHasChildren(node) && !node_just_created ) {
              // current node does not have children AND it is not a new node 
              // -> expand pruned node
              this->expandNode(node);
            }
            else {
              // not a pruned node, create requested child
              this->createNodeChild(node, pos);
              created_node = true;
            }
          }

          if (lazy_eval)
            return updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, occupied, class_obs, color_obs, lazy_eval);
          else {
            SemanticOcTreeNode<SEMANTICS>* retval = updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, occupied, class_obs, color_obs, lazy_eval);
            // prune node if possible, otherwise set own probability
            // note: combining both did not lead to a speedup!
            if (this->pruneNode(node)){
              // return pointer to current parent (pruned), the just updated node no longer exists
              retval = node;
            } else {
              node->updateSemanticsChildren();
              node->updateColorChildren();
            }

            return retval;
          }
        }

        // at last level, update node, end of recursion
        else {
          if (this->use_change_detection) {
            bool occBefore = this->isNodeOccupied(node);
            updateNodeLogOdds(node, occupied, class_obs, color_obs);

            if (node_just_created){  // new node
              this->changed_keys.insert(std::pair<OcTreeKey,bool>(key, true));
            } else if (occBefore != this->isNodeOccupied(node)) {  // occupancy changed, track it
              KeyBoolMap::iterator it = this->changed_keys.find(key);
              if (it == this->changed_keys.end())
                this->changed_keys.insert(std::pair<OcTreeKey,bool>(key, false));
              else if (it->second == false)
                this->changed_keys.erase(it);
            }
          } else {
            updateNodeLogOdds(node, occupied, class_obs, color_obs);
          }
          return node;
        }
    }

    template<class SEMANTICS>
    void SemanticOcTree<SEMANTICS>::updateNodeLogOdds(SemanticOcTreeNode<SEMANTICS>* node, bool occupied,
                                                      const ColorOcTreeNode::Color& class_obs,
                                                      const ColorOcTreeNode::Color& color_obs)
    {
        // node color update
        averageNodeColor(node, color_obs.r, color_obs.g, color_obs.b);
        // node semantics and occupancy update
        if(occupied)
        {
            updateNodeSemantics(node, class_obs);
        } else {
            updateFreeNode(node);
        }
    }

    template<class SEMANTICS>
    void SemanticOcTree<SEMANTICS>::insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                                                     double maxrange, bool discretize)
    {
        KeySet free_cells, occupied_cells;
        if (discretize)
          this->computeDiscreteUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);
        else
          this->computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);

        // Insert data into tree only for the free cells, occupied cells will be taken care of separately  -----------------------
        for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
          updateNode(*it, false);
        }
    }

    template<class SEMANTICS>
    void SemanticOcTree<SEMANTICS>::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    template<class SEMANTICS>
    void SemanticOcTree<SEMANTICS>::updateInnerOccupancyRecurs(SemanticOcTreeNode<SEMANTICS>* node, unsigned int depth) {
        // Only recurse and update for inner nodes:
        if (this->nodeHasChildren(node)){
            // Return early for last level:
            if (depth < this->tree_depth){
                for (unsigned int i=0; i<8; i++) {
                    if (this->nodeChildExists(node, i)) {
                        updateInnerOccupancyRecurs(this->getNodeChild(node, i), depth+1);
                    }
                }
            }
            // Update occupancy, semantics and color for inner nodes
            node->updateSemanticsChildren();
            node->updateColorChildren();
        }
    }
    
    template<class SEMANTICS>
    bool SemanticOcTree<SEMANTICS>::checkNeedsUpdate(const SemanticOcTreeNode<SEMANTICS>* node, bool occupied, const ColorOcTreeNode::Color& class_obs)
    {   
        SEMANTICS sem = node->getSemantics();
        
        if((!occupied && node->getLogOdds() <= minOccupancyLogOdds) ||
           (sem.data[0].color == class_obs &&
            sem.data[0].logOdds >= maxLogOddsTree && sem.data[1].logOdds <= minLogOddsTree && 
            sem.data[2].logOdds <= minLogOddsTree && sem.others <= minLogOddsTree))
        {
            return false;
        } else {
            return true;
        }
    }
    
    template<class SEMANTICS>
    bool SemanticOcTree<SEMANTICS>::get_ray_RLE(const octomap::point3d& origin,
                                                const octomap::point3d& end,
                                                semantic_octomap::RayRLE& rayRLE_msg)
    {
        KeyRay* keyray = &(this->keyrays.at(0));
        if (this->computeRayKeys(origin, end, *keyray))
        {   
            semantic_octomap::LE le_msg_prev;
            le_msg_prev.le.push_back(1);
            SemanticOcTreeNode<SEMANTICS>* prevNode = this->search(*(keyray->begin()));
            if (!prevNode) 
            {
                for (int i = 0; i < 4; i++)
                {
                    le_msg_prev.le.push_back(-0.1);
                }
            } else if (!prevNode->isSemanticsSet()) {
                float l = prevNode->getLogOdds() - log(4.);
                for (int i = 0; i < 4; i++)
                {
                    le_msg_prev.le.push_back(l);
                }
            } else {
                SEMANTICS nodeSem = prevNode->getSemantics();
                for (int i = 0; i < 3; i++)
                {
                    le_msg_prev.le.push_back(nodeSem.data[i].logOdds);
                }
                le_msg_prev.le.push_back(nodeSem.others);
            }

            for (KeyRay::iterator it = keyray->begin()+1; it != keyray->end(); ++it)
            {
                SemanticOcTreeNode<SEMANTICS>* currNode = this->search(*it);
                
                if (!currNode && !prevNode)
                {
                    le_msg_prev.le[0] += 1;
                } else if (currNode && prevNode) {
                    if (*currNode == *prevNode)
                    {
                        le_msg_prev.le[0] += 1;
                    } else {
                        rayRLE_msg.le_list.push_back(le_msg_prev);
                        le_msg_prev.le.clear();
                        le_msg_prev.le.push_back(1);
                        
                        if (!currNode->isSemanticsSet()) {
                            float l = currNode->getLogOdds() - log(4.);
                            for (int i = 0; i < 4; i++)
                            {
                                le_msg_prev.le.push_back(l);
                            }
                        } else {
                            SEMANTICS nodeSem = currNode->getSemantics();
                            for (int i = 0; i < 3; i++)
                            {
                                le_msg_prev.le.push_back(nodeSem.data[i].logOdds);
                            }
                            le_msg_prev.le.push_back(nodeSem.others);
                        }
                        prevNode = currNode;
                    }
                } else {
                    rayRLE_msg.le_list.push_back(le_msg_prev);
                    le_msg_prev.le.clear();
                    le_msg_prev.le.push_back(1);
                    
                    if (!currNode)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            le_msg_prev.le.push_back(-0.1);
                        }
                    } else {
                        if (!currNode->isSemanticsSet()) {
                            float l = currNode->getLogOdds() - log(4.);
                            for (int i = 0; i < 4; i++)
                            {
                                le_msg_prev.le.push_back(l);
                            }
                        } else {
                            SEMANTICS nodeSem = currNode->getSemantics();
                            for (int i = 0; i < 3; i++)
                            {
                                le_msg_prev.le.push_back(nodeSem.data[i].logOdds);
                            }
                            le_msg_prev.le.push_back(nodeSem.others);
                        }
                    }
                    
                    prevNode = currNode;
                }
            }
            rayRLE_msg.le_list.push_back(le_msg_prev);
            
            return true;
        } else {
            return false;
        }
    }

    template<class SEMANTICS>
    typename SemanticOcTree<SEMANTICS>::StaticMemberInitializer SemanticOcTree<SEMANTICS>::semanticOcTreeMemberInit;

} // end namespace

#endif //SEMANTIC_OCTOMAP_SEMANTICOCTREE_HXX
