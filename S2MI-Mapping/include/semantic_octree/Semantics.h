#ifndef SEMANTIC_OCTOMAP_SEMANTICS_H
#define SEMANTIC_OCTOMAP_SEMANTICS_H

#include <octomap/ColorOcTree.h>
#include <math.h>
#define NUM_SEMANTICS 3
#define LOGINIT -0.1


namespace octomap
{

    /// Structure of semantic color with log-odds
    struct ColorWithLogOdds
    {
        ColorWithLogOdds()
        {
            color = ColorOcTreeNode::Color(255,255,255);
            logOdds = LOGINIT;
        }

        ColorWithLogOdds(ColorOcTreeNode::Color col, float log_odds)
        {
            color = col;
            logOdds = log_odds;
        }

        ColorOcTreeNode::Color color;
        float logOdds;

        inline bool operator==(const ColorWithLogOdds& rhs) const
        {
            return color == rhs.color && logOdds == rhs.logOdds;
        }

        inline bool operator!=(const ColorWithLogOdds& rhs) const
        {
            return color != rhs.color || logOdds != rhs.logOdds;
        }

        inline bool operator<(const ColorWithLogOdds& rhs) const
        {
            return logOdds < rhs.logOdds;
        }

        inline bool operator>(const ColorWithLogOdds& rhs) const
        {
            return logOdds > rhs.logOdds;
        }
    };

    std::ostream& operator<<(std::ostream& out, ColorWithLogOdds const& c);

    /// Structure contains semantic colors and their log-odds
    struct SemanticsLogOdds
    {
        ColorWithLogOdds data[NUM_SEMANTICS]; ///<Semantic colors and log-odds, ordered by log-odds
        float others;

        SemanticsLogOdds()
        {
            for(int i = 0; i < NUM_SEMANTICS; i++)
            {
                data[i] = ColorWithLogOdds();
            }
            others = LOGINIT;
        }

        bool operator==(const SemanticsLogOdds& rhs) const
        {
            for(int i = 0; i < NUM_SEMANTICS; i++)
            {
                if(data[i] != rhs.data[i])
                {
                    return false;
                }
            }
            if(others != rhs.others)
            {
                return false;
            }
            return true;
        }

        bool operator!=(const SemanticsLogOdds& rhs) const
        {
            return !(*this == rhs);
        }

        ColorOcTreeNode::Color getSemanticColor() const
        {
            return data[0].color;
        }

        bool isSemanticsSet() const
        {
            for(int i = 0; i < NUM_SEMANTICS; i++)
            {
                if(data[i].color != ColorOcTreeNode::Color(255,255,255))
                    return true;
            }
            return false;
        }

        /// initialize semantics from observation
        static SemanticsLogOdds initSemantics(ColorOcTreeNode::Color obs, float value,
                                              float phi, float psi, float maxLogOdds, float minLogOdds);

        /// Perform fusion between two semantic log-odds
        static SemanticsLogOdds semanticFusion(const SemanticsLogOdds l1, const SemanticsLogOdds l2);

        /// Perform semantic fusion between a semantic log-odds and an observation
        static SemanticsLogOdds fuseObs(const SemanticsLogOdds l, const ColorOcTreeNode::Color obs,
                                        float phi, float psi, float maxLogOdds, float minLogOdds);
                                        
        /// Perform semantic fusion between a semantic log-odds and a free observation
        static SemanticsLogOdds fuseObsFree(const SemanticsLogOdds l, float phi, float minLogOdds);
        
        /// Obtain occupancy from semantic log-odds
        static inline float getOccFromSem(const SemanticsLogOdds l) {
            float occOdds = (float) exp(l.others);
            for(int i = 0; i < 3; i++)
            {
                if(l.data[i].color != ColorOcTreeNode::Color(255,255,255))
                    occOdds += (float) exp(l.data[i].logOdds);
            }
            return log(occOdds);
        }
    };

    std::ostream& operator<<(std::ostream& out, SemanticsLogOdds const& l);

    //std::vector<float> softmax(const SemanticsBayesian l);

}
#endif //SEMANTIC_OCTOMAP_SEMANTICS_H
