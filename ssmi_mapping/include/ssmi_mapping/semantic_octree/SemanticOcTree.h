#ifndef SEMANTIC_OCTOMAP_SEMANTICOCTREE_H
#define SEMANTIC_OCTOMAP_SEMANTICOCTREE_H

#include <iostream>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap_utils.h>
#include <ssmi_mapping/semantic_octree/SemanticOcTreeNode.h>

#include "ssmi_interface/msg/le.hpp"
#include "ssmi_interface/msg/ray_rle.hpp"

namespace octomap
{

    /// Tree definition
    template<class SEMANTICS>
    class SemanticOcTree : public OccupancyOcTreeBase<SemanticOcTreeNode<SEMANTICS> >
{
    public:

    /// Default constructor, sets resolution of leafs
    SemanticOcTree(double resolution);

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    SemanticOcTree* create() const {return new SemanticOcTree(this->resolution);}

    std::string getTreeType() const {return "ColorOcTree";} // For display in rviz

    inline float getPhi() const {return phiTree;}

    inline float getPsi() const {return psiTree;}

    inline float getMaxLogOdds() const {return maxLogOddsTree;}

    inline float getMinLogOdds() const {return minLogOddsTree;}

    inline void setPhi(float phi) {phiTree = phi;}

    inline void setPsi(float psi) {psiTree = psi;}

    inline void setMaxLogOdds(float maxProb) {maxLogOddsTree = logodds(maxProb);}

    inline void setMinLogOdds(float minProb)
    {
        minLogOddsTree = logodds(minProb);
        minOccupancyLogOdds = log(4.) + minLogOddsTree;
    }

    /**
    * Prunes a node when it is collapsible. This overridden
    * version only considers the node occupancy and color for pruning,
    * The confidence is average of children's confidence after pruning.
    * \return true if pruning was successful
    */
    virtual bool pruneNode(SemanticOcTreeNode<SEMANTICS>* node);

    virtual bool isNodeCollapsible(const SemanticOcTreeNode<SEMANTICS>* node) const;

    bool isUseSemanticColor(){return this->root->use_semantic_color;}
    
    bool doesWriteSemantics(){return this->root->write_semantics;}

    void setUseSemanticColor(bool use);
    
    void setWriteSemantics(bool write);

    // set node color at given key or coordinate. Replaces previous color.
    SemanticOcTreeNode<SEMANTICS>* setNodeColor(const OcTreeKey& key, uint8_t r,
                                                uint8_t g, uint8_t b);

    SemanticOcTreeNode<SEMANTICS>* setNodeColor(float x, float y,
                                                float z, uint8_t r,
                                                uint8_t g, uint8_t b) {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
        return setNodeColor(key,r,g,b);
    }


    /// Integrate color measurement (RGB) at given node. Average with previous color
    SemanticOcTreeNode<SEMANTICS>* averageNodeColor(SemanticOcTreeNode<SEMANTICS>* node, uint8_t r,
                                                    uint8_t g, uint8_t b);

    /// Integrate color measurement (RGB) at given key. Average with previous color
    SemanticOcTreeNode<SEMANTICS>* averageNodeColor(const OcTreeKey& key, uint8_t r,
                                                    uint8_t g, uint8_t b) {
        SemanticOcTreeNode<SEMANTICS>* node = this->search(key);
        return averageNodeColor(node, r, g, b);
    }

    /// Integrate color measurement (RGB) at given coordinate. Average with previous color
    SemanticOcTreeNode<SEMANTICS>* averageNodeColor(float x, float y,
                                                    float z, uint8_t r,
                                                    uint8_t g, uint8_t b) {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
        return averageNodeColor(key,r,g,b);
    }

    /// Update semantics for given node from a new observation by doing bayesian fusion
    SemanticOcTreeNode<SEMANTICS>* updateNodeSemantics(SemanticOcTreeNode<SEMANTICS>* node, ColorOcTreeNode::Color obs);

    /// Update semantics for given key from a new observation by doing bayesian fusion
    SemanticOcTreeNode<SEMANTICS>* updateNodeSemantics(const OcTreeKey& key, ColorOcTreeNode::Color obs) {
        SemanticOcTreeNode<SEMANTICS>* node = this->search(key);
        return updateNodeSemantics(node, obs);
    }

    /// Update semantics at given coordinates from a new observation by doing bayesian fusion
    SemanticOcTreeNode<SEMANTICS>* updateNodeSemantics(float x, float y, float z, ColorOcTreeNode::Color obs) {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
        return updateNodeSemantics(key, obs);
    }

    /// Update logodds for a given node which is observed as free
    SemanticOcTreeNode<SEMANTICS>* updateFreeNode(SemanticOcTreeNode<SEMANTICS>* node);

    /// Update node from an incoming semantic octomap by doing bayesian fusion
    virtual SemanticOcTreeNode<SEMANTICS>* updateNode(const OcTreeKey& key, float node_value,
                                                      const ColorOcTreeNode::Color& node_color,
                                                      const SEMANTICS& node_semantics,
                                                      float consensus_weight, bool lazy_eval = false);

    /// Update node from an incoming semantic octomap by doing bayesian fusion
    virtual SemanticOcTreeNode<SEMANTICS>* updateNode(float x, float y, float z, float node_value,
                                                      const ColorOcTreeNode::Color& node_color,
                                                      const SEMANTICS& node_semantics,
                                                      float consensus_weight, bool lazy_eval = false);

    /// Update node from a new observation by doing bayesian fusion
    virtual SemanticOcTreeNode<SEMANTICS>* updateNode(const OcTreeKey& key, bool occupied,
                                                      const ColorOcTreeNode::Color& class_obs = ColorOcTreeNode::Color(255,255,255),
                                                      const ColorOcTreeNode::Color& color_obs = ColorOcTreeNode::Color(255,255,255),
                                                      bool lazy_eval = false);

    /// Update node from a new observation by doing bayesian fusion
    virtual SemanticOcTreeNode<SEMANTICS>* updateNode(float x, float y, float z, bool occupied,
                                                      const ColorOcTreeNode::Color& class_obs = ColorOcTreeNode::Color(255,255,255),
                                                      const ColorOcTreeNode::Color& color_obs = ColorOcTreeNode::Color(255,255,255),
                                                      bool lazy_eval = false);

    /// Update semantic logodds value of node by fusing the incoming node
    virtual void updateNodeLogOdds(SemanticOcTreeNode<SEMANTICS>* node, const ColorOcTreeNode::Color& node_color,
                                   const SEMANTICS& output_sem, const float& output_value);

    /// Update semantic logodds value of node by adding the new observation
    virtual void updateNodeLogOdds(SemanticOcTreeNode<SEMANTICS>* node, bool occupied,
                                   const ColorOcTreeNode::Color& class_obs,
                                   const ColorOcTreeNode::Color& color_obs);

    virtual void insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                                  double maxrange=-1., bool discretize = false);

    // Update inner nodes' occupancy, RGB color and semantics
    void updateInnerOccupancy();
    
    bool get_ray_RLE(const octomap::point3d& origin, const octomap::point3d& end, ssmi_interface::msg::RayRLE& rayRLE_msg);

    protected:
    void updateInnerOccupancyRecurs(SemanticOcTreeNode<SEMANTICS>* node, unsigned int depth);
    
    SemanticOcTreeNode<SEMANTICS>* updateNodeRecurs(SemanticOcTreeNode<SEMANTICS>* node, bool node_just_created, const OcTreeKey& key,
                                                    unsigned int depth, bool occupied, const ColorOcTreeNode::Color& class_obs,
                                                    const ColorOcTreeNode::Color& color_obs, bool lazy_eval = false);

    SemanticOcTreeNode<SEMANTICS>* updateNodeRecurs(SemanticOcTreeNode<SEMANTICS>* node, bool node_just_created, const OcTreeKey& key,
                                                    unsigned int depth, const ColorOcTreeNode::Color& node_color,
                                                    const SEMANTICS& output_sem, const float& output_value, bool lazy_eval = false);
    
    bool checkNeedsUpdate(const SemanticOcTreeNode<SEMANTICS>* node, bool occupied, const ColorOcTreeNode::Color& class_obs);
    
    bool checkNeedsUpdate(const SemanticOcTreeNode<SEMANTICS>* node, float node_value, const SEMANTICS& node_semantics,
                          float consensus_weight, SEMANTICS& output_sem, float& output_value);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
        StaticMemberInitializer() {
            SemanticOcTree* tree = new SemanticOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

        /**
        * Dummy function to ensure that MSVC does not drop the
        * StaticMemberInitializer, causing this tree failing to register.
        * Needs to be called from the constructor of this octree.
        */
        void ensureLinking() {};
    };

    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer semanticOcTreeMemberInit;

    protected:
    float phiTree, psiTree;
    float maxLogOddsTree, minLogOddsTree;
    float minOccupancyLogOdds;
};

} // end namespace

// Implementation
#include <ssmi_mapping/semantic_octree/SemanticOcTree.hxx>

#endif //SEMANTIC_OCTOMAP_SEMANTICOCTREE_H
