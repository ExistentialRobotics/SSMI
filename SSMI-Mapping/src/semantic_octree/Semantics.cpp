#include <semantic_octree/Semantics.h>
#include <vector>
#include <algorithm>

#define logALPHA static_cast<float>(log(0.8))
#define log_minus_ALPHA static_cast<float>(log(0.2))

#define logBETA static_cast<float>(log(0.1))
#define log_minus_BETA static_cast<float>(log(0.9))

namespace octomap
{
    // Struct ColorWithLogOdds implementation -------------------------------------
    std::ostream& operator<<(std::ostream& out, ColorWithLogOdds const& c)
    {
        return out << '(' << c.color << ' ' << c.logOdds << ')';
    }

    // Struct SemanticsLogOdds implementation  --------------------------------------
    SemanticsLogOdds SemanticsLogOdds::initSemantics(const ColorOcTreeNode::Color& obs, float value,
                                                     float phi, float psi, float maxLogOdds, float minLogOdds)
    {
        SemanticsLogOdds output;
        output.data[0] = ColorWithLogOdds(obs, logBETA + value + psi);
        if(output.data[0].logOdds < minLogOdds)
        {
            output.data[0].logOdds = minLogOdds;
        } else if(output.data[0].logOdds > maxLogOdds)
        {
            output.data[0].logOdds = maxLogOdds;
        }
        
        output.others = log_minus_BETA + value + phi;
        if(output.others < minLogOdds)
        {
            output.others = minLogOdds;
        }
        
        return output;
    }


    SemanticsLogOdds SemanticsLogOdds::semanticFusion(const SemanticsLogOdds& l1, const SemanticsLogOdds& l2)
    {
        std::vector<ColorOcTreeNode::Color> v1, v2, v3;
        for(int i = 0; i < NUM_SEMANTICS; i++)
        {
            if(l1.data[i].color != ColorOcTreeNode::Color(255,255,255))
                v1.push_back(l1.data[i].color);

            if(l2.data[i].color != ColorOcTreeNode::Color(255,255,255))
                v2.push_back(l2.data[i].color);
        }

        v3 = v1;
        for(int i = 0; i < v2.size(); i++)
        {
            bool exists = false;
            for(int j = 0; j < v1.size(); j++)
            {
                if(v2[i] == v1[j])
                {
                    exists = true;
                    break;
                }
            }
            if(exists == false)
                v3.push_back(v2[i]);
        }

        std::vector<ColorWithLogOdds> avg_data(v3.size());
        float others1 = l1.others - log(v3.size() - v1.size() + 1);
        float others2 = l2.others - log(v3.size() - v2.size() + 1);

        for(int i = 0; i < v3.size(); i++)
        {
            avg_data[i].color = v3[i];
            float logOdds2;
            bool assigned1 = false, assigned2 = false;
            for(int j = 0; j < NUM_SEMANTICS; j++)
            {
                if(avg_data[i].color == l1.data[j].color)
                {
                    avg_data[i].logOdds = l1.data[j].logOdds;
                    assigned1 = true;
                }

                if(avg_data[i].color == l2.data[j].color)
                {
                    logOdds2 = l2.data[j].logOdds;
                    assigned2 = true;
                }
            }

            if(!assigned1)
                avg_data[i].logOdds = others1;

            if(!assigned2)
                logOdds2 = others2;

            avg_data[i].logOdds = (avg_data[i].logOdds + logOdds2) / 2;
        }
        others1 = (others1 + others2) / 2;

        std::sort(avg_data.begin(), avg_data.end()); // Ascending sort
        SemanticsLogOdds output;
        if(avg_data.size() <= NUM_SEMANTICS)
        {
            for(int i = 0; i < avg_data.size(); i++)
                output.data[i] = avg_data[avg_data.size() - 1 - i];

            output.others = others1;
        } else {
            float exp_others = exp(others1);
            for(int i = 0; i < avg_data.size(); i++)
            {
                if(i < NUM_SEMANTICS)
                {
                    output.data[i] = avg_data[avg_data.size() - 1 - i];
                } else {
                    exp_others += exp(avg_data[avg_data.size() - 1 - i].logOdds);
                }
            }

            output.others = log(exp_others);
        }

        return output;
    }
    
    SemanticsLogOdds SemanticsLogOdds::semanticFusion(const SemanticsLogOdds& l1, const SemanticsLogOdds& l2,
                                                      float consensus_weight, float maxLogOdds, float minLogOdds)
    {
        std::vector<ColorOcTreeNode::Color> v1, v2, v3;
        for(int i = 0; i < NUM_SEMANTICS; i++)
        {
            if(l1.data[i].color != ColorOcTreeNode::Color(255,255,255))
                v1.push_back(l1.data[i].color);

            if(l2.data[i].color != ColorOcTreeNode::Color(255,255,255))
                v2.push_back(l2.data[i].color);
        }

        v3 = v1;
        for(int i = 0; i < v2.size(); i++)
        {
            bool exists = false;
            for(int j = 0; j < v1.size(); j++)
            {
                if(v2[i] == v1[j])
                {
                    exists = true;
                    break;
                }
            }
            if(exists == false)
                v3.push_back(v2[i]);
        }

        std::vector<ColorWithLogOdds> avg_data(v3.size());
        float others1 = l1.others - log(v3.size() - v1.size() + 1);
        float others2 = l2.others - log(v3.size() - v2.size() + 1);

        for(int i = 0; i < v3.size(); i++)
        {
            avg_data[i].color = v3[i];
            float logOdds2;
            bool assigned1 = false, assigned2 = false;
            for(int j = 0; j < NUM_SEMANTICS; j++)
            {
                if(avg_data[i].color == l1.data[j].color)
                {
                    avg_data[i].logOdds = l1.data[j].logOdds;
                    assigned1 = true;
                }

                if(avg_data[i].color == l2.data[j].color)
                {
                    logOdds2 = l2.data[j].logOdds;
                    assigned2 = true;
                }
            }

            if(!assigned1)
                avg_data[i].logOdds = others1;

            if(!assigned2)
                logOdds2 = others2;

            avg_data[i].logOdds = avg_data[i].logOdds + consensus_weight * logOdds2;
        }
        others1 = others1 + consensus_weight * others2;

        std::sort(avg_data.begin(), avg_data.end()); // Ascending sort
        SemanticsLogOdds output;
        if(avg_data.size() <= NUM_SEMANTICS)
        {
            for(int i = 0; i < avg_data.size(); i++)
            {
                output.data[i].color = avg_data[avg_data.size() - 1 - i].color;
                output.data[i].logOdds = clipLogOdds(avg_data[avg_data.size() - 1 - i].logOdds, maxLogOdds, minLogOdds);
            }

            output.others = clipLogOdds(others1, maxLogOdds, minLogOdds);
        } else {
            float exp_others = exp(others1);
            for(int i = 0; i < avg_data.size(); i++)
            {
                if(i < NUM_SEMANTICS)
                {
                    output.data[i].color = avg_data[avg_data.size() - 1 - i].color;
                    output.data[i].logOdds = clipLogOdds(avg_data[avg_data.size() - 1 - i].logOdds, maxLogOdds, minLogOdds);
                } else {
                    exp_others += exp(avg_data[avg_data.size() - 1 - i].logOdds);
                }
            }

            output.others = clipLogOdds(log(exp_others), maxLogOdds, minLogOdds);
        }

        return output;
    }

    SemanticsLogOdds SemanticsLogOdds::semanticFusionInit(float value, const SemanticsLogOdds& node_semantics, float consensus_weight,
                                                          float maxLogOdds, float minLogOdds, bool hostSemIsInit)
    {
        SemanticsLogOdds sem;
        ColorOcTreeNode::Color white_color(255,255,255);
        float non_whites = 0;
        for (int i = 0; i < NUM_SEMANTICS; i++)
        {
            if (node_semantics.data[i].color != white_color)
            {
                sem.data[i].color = node_semantics.data[i].color;
                non_whites += 1;
            } else {break;}
        }
        float class_logOdds = value - log(non_whites + 1);
        for (int i = 0; i < (int)non_whites; i++)
            sem.data[i].logOdds = class_logOdds;
        sem.others = class_logOdds;
        
        if (hostSemIsInit)
            return semanticFusion(node_semantics, sem, consensus_weight, maxLogOdds, minLogOdds);
        else
            return semanticFusion(sem, node_semantics, consensus_weight, maxLogOdds, minLogOdds);
    }

    SemanticsLogOdds SemanticsLogOdds::fuseObs(const SemanticsLogOdds& l, const ColorOcTreeNode::Color& obs,
                             float phi, float psi, float maxLogOdds, float minLogOdds)
    {
        std::vector<ColorWithLogOdds> v;
        float others = l.others;
        bool isOthers = true;
        // check for occupancy only measurement (145,145,145)
        if(obs == ColorOcTreeNode::Color(145,145,145))
        {
            try{
                //increment log odds of others and current occupancy
                others += psi * 0.75;
                if(others > (maxLogOdds * 0.75)){
                    others = maxLogOdds * 0.75;
                }

                for(int i = 0; i < NUM_SEMANTICS; i++)
                {   
                    if(l.data[i].color == ColorOcTreeNode::Color(255,255,255)) //uninitialized class
                        continue;

                    if(l.data[i].color == ColorOcTreeNode::Color(145,145,145)) //dont add occupancy to "occupied-unknown_label" class
                        v.push_back(l.data[i]);
                        continue;
                    
                    v.push_back(l.data[i]);
                    v.back().logOdds += psi * 0.75;    
                    if(v.back().logOdds > maxLogOdds * 0.75){
                        v.back().logOdds = maxLogOdds * 0.75;
                    }
                }

                std::sort(v.begin(), v.end());
                SemanticsLogOdds output;
                output.others = others;
                for(int i = 0; i < v.size(); i++){
                    output.data[i] = v[v.size() - 1 - i];
                }

                return output;
            }catch (const std::exception& e) {
                // Catch any standard exception
                std::cerr << "Caught exception: " << e.what() << std::endl;
            } catch (...) {
                // Catch any other type of exception (generic catch)
                std::cerr << "Caught unknown exception!" << std::endl;
            }

        }

        for(int i = 0; i < NUM_SEMANTICS; i++)
        {
            if(l.data[i].color != ColorOcTreeNode::Color(255,255,255))
            {
                v.push_back(l.data[i]);
                
                if(obs == l.data[i].color)
                {
                    isOthers = false;
                    v.back().logOdds += psi;
                    if(v.back().logOdds > maxLogOdds)
                        v.back().logOdds = maxLogOdds;
                } else {
                    v.back().logOdds += phi;
                    if(v.back().logOdds < minLogOdds)
                        v.back().logOdds = minLogOdds;
                }
            }
        }

        if(isOthers)
        {
            ColorWithLogOdds aux(obs, l.others + logALPHA + psi);
            v.push_back(aux);
            others += log_minus_ALPHA + phi;

            std::sort(v.begin(), v.end());
            
            if(v.size() > NUM_SEMANTICS)
                {
                    others = log(exp(others) + exp(v[0].logOdds));
                    v.erase(v.begin());
                }
            
        } else {
            others += phi;

            std::sort(v.begin(), v.end());
        }

        if(others < minLogOdds)
        {
            others = minLogOdds;
        } else if(others > maxLogOdds)
        {
            others = maxLogOdds;
        }

        SemanticsLogOdds output;
        output.others = others;
        for(int i = 0; i < v.size(); i++)
            output.data[i] = v[v.size() - 1 - i];

        return output;
    }
    
    SemanticsLogOdds SemanticsLogOdds::fuseObsFree(const SemanticsLogOdds& l, float phi, float minLogOdds)
    {
        SemanticsLogOdds output = l;
        
        for(int i = 0; i < NUM_SEMANTICS; i++)
        {
            if(output.data[i].color != ColorOcTreeNode::Color(255,255,255))
            {
                output.data[i].logOdds += phi;
                if(output.data[i].logOdds < minLogOdds)
                    output.data[i].logOdds = minLogOdds;
            }
        }
        
        output.others += phi;
        if(output.others < minLogOdds)
            output.others = minLogOdds;
        
        return output;
    }

    std::ostream& operator<<(std::ostream& out, SemanticsLogOdds const& s) {
        out << '(';
        for(int i = 0; i < NUM_SEMANTICS - 1; i++)
            out << s.data[i] << ' ';
        out << s.data[NUM_SEMANTICS - 1];
        out << ')';
        return out;
    }

} //namespace octomap
