#ifndef SEMANTIC_OCTOMAP_SEMANTICOCTREENODE_H
#define SEMANTIC_OCTOMAP_SEMANTICOCTREENODE_H

#include <semantic_octree/SemanticOcTree.h>

namespace octomap
{

    // Forward declaration for "friend"
    template <class SEMANTICS> class SemanticOcTree;

    /// Node definition
    template <class SEMANTICS>
    class SemanticOcTreeNode : public ColorOcTreeNode {
    public:
        friend class SemanticOcTree<SEMANTICS>; // Needs access to node children (inherited)

    public:

        /// Default constructor
        SemanticOcTreeNode() : ColorOcTreeNode(), semantics(), use_semantic_color(true){}

        /// Copy constructor
        SemanticOcTreeNode(const SemanticOcTreeNode& rhs)
        {
            copyData(rhs);
        }

        /// Operator
        inline bool operator==(const SemanticOcTreeNode& rhs) const{
            return (rhs.value == value && rhs.semantics == semantics);
        }

        /// Copy data
        void copyData(const SemanticOcTreeNode& from)
        {
            ColorOcTreeNode::copyData(from);
            semantics = from.getSemantics();
        }

        /// Get semantics
        inline SEMANTICS getSemantics() const {return semantics;}

        /// Set semantics
        inline void setSemantics(SEMANTICS from){semantics = from;}

        /// Is semantics set: not set if all colors are pure white
        inline bool isSemanticsSet() const;

        /// Update semantics (colors and log-odds) from children by doing semantic fusion (using method in template class)
        void updateSemanticsChildren();

        /// Do semantic fusion for children nodes (using method in template class)
        SEMANTICS getFusedChildSemantics() const;

        /// Read from file
        std::istream& readData(std::istream &s);

        /// Write to file, also used to serialize octomap, we change the displayed color here
        std::ostream& writeData(std::ostream &s) const;

    protected:
        SEMANTICS semantics;
        bool use_semantic_color; ///<Whether use semantic color rather than rgb color
    };


    // Node implementation  --------------------------------------
    template<class SEMANTICS>
    bool SemanticOcTreeNode<SEMANTICS>::isSemanticsSet() const
    {
        return this->semantics.isSemanticsSet();
    }

    template<class SEMANTICS>
    void SemanticOcTreeNode<SEMANTICS>::updateSemanticsChildren()
    {
        semantics = getFusedChildSemantics();
        float logOddsValue = SEMANTICS::getOccFromSem(semantics);
        this->setLogOdds(logOddsValue);
    }

    template<class SEMANTICS>
    SEMANTICS SemanticOcTreeNode<SEMANTICS>::getFusedChildSemantics() const
    {
        // Fuse semantics of children node by semantic fusion
        SEMANTICS sem;
        bool fusion_started = false;
        if(children != NULL)
        {
            for(int i = 0; i < 8; i++)
            {
                SemanticOcTreeNode* child =  static_cast<SemanticOcTreeNode*>(children[i]);
                if(child != NULL && child->isSemanticsSet())
                {
                    if(fusion_started)
                        sem = SEMANTICS::semanticFusion(sem, child->getSemantics());
                    else
                    {
                        sem = child->getSemantics();
                        fusion_started = true;
                    }
                }
            }
        }
        return sem;
    }

    template<class SEMANTICS>
    std::istream& SemanticOcTreeNode<SEMANTICS>::readData(std::istream &s) {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &color, sizeof(Color)); // color
        return s;
    }

    template<class SEMANTICS>
    std::ostream& SemanticOcTreeNode<SEMANTICS>::writeData(std::ostream &s) const {
        //TODO adapt to show semantic colors
        s.write((const char*) &value, sizeof(value)); // occupancy
        if(use_semantic_color)
        {
            Color sem_color = semantics.getSemanticColor();
            s.write((const char*) &sem_color, sizeof(Color)); // semantic color
        }
        else
            s.write((const char*) &color, sizeof(Color)); // color
        return s;
    }
} // namespace octomap

#endif //SEMANTIC_OCTOMAP_SEMANTICOCTREENODE_H
