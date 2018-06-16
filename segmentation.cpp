#include "my_header.h"
#include <iostream>
using namespace std;

const float threshold_one = 0.05f; /* kept threshold_one/100 of the detected edges */
const float threshold_two = 0.05f; /* will not continue connecting if weight < max_string_length * threshold_two */
const int min_feature_length = 15;
const int max_string_length = 5;
const float VERY_SMALL_SHARPNESS = -10;

/* Normal vectors computed by Assimp are always unit-length. 
   However, this needn't apply for normals that have been taken directly from the model file. */
void normalize_normals(const aiMesh *m) {
    aiVector3D* ns = m->mNormals;
    for(int i = 0; i < m->mNumVertices; i++) {
        ns[i] = ns[i].NormalizeSafe();
    }
}

void gen_face_normals(const aiMesh *mesh, face_normals &fn) {
    unsigned int f,i;
    fn = new aiVector3D[mesh->mNumFaces];
    for (f = 0; f < mesh->mNumFaces; ++f) {
        const aiFace* face = &mesh->mFaces[f];
        aiVector3D norm = aiVector3D(0,0,0);
        for(i = 0; i < face->mNumIndices; i++) {
            norm += mesh->mNormals[face->mIndices[i]];
        }
        fn[f] = norm.Normalize();
    }
}

void connect_to_others(seg_edge_list &edges, int index, vector<int> &result, int len, float weight) {
    float max_sharpness = VERY_SMALL_SHARPNESS;
    int new_index = -1;
    int repeat = edges.size();
    for(int i=0; i<repeat; i++) {
        if(edges[index].canLinkTo(edges[i])!=NO_LINK && edges[index].e!=edges[i].e) { /* neighbors */
            float s = edges[i].sharpness;
            if(s>max_sharpness) {
                max_sharpness = s;
                new_index = i;
            }
        }
    }
    //cout<<"s"<<max_sharpness<<" ";
    
    if(new_index<0) { /* can not find a new edge */
        for(int i=0; i<len; i++)
            result.pop_back();
        return;
    }

    /* link a and new_index */
    result.push_back(new_index);
    link_type lt = edges[index].canLinkTo(edges[new_index]);
    switch(lt)
    {
        case LINK_AA: { edges[index].islockedA = edges[new_index].islockedA = true; break; }
        case LINK_AB: { edges[index].islockedA = edges[new_index].islockedB = true; break; }
        case LINK_BA: { edges[index].islockedB = edges[new_index].islockedA = true; break; }
        case LINK_BB: { edges[index].islockedB = edges[new_index].islockedB = true; break; }
    }

    if(edges[index].isChosen) {
        weight = 0;
        len = 0;
    }
    else {
        weight += max_sharpness;
        len++;
    }
    if(len<max_string_length) {
        connect_to_others(edges,new_index,result,len,weight);
    }
    else if(weight < max_string_length*threshold_two) {
        for(int i=0; i<max_string_length; i++) {
            result.pop_back();
        }
    }
}

void expand_features(seg_edge_list &edges, edge_list &result) {
    int sum = 0;
    for(seg_edge_list::iterator it = edges.begin(); it != edges.end(); it++) {
        if(!it->isChosen)
            continue;
        sum++;
        vector<int> S;
        connect_to_others(edges, it-edges.begin(), S, 0, 0);
        cout<<S.size()<<" ";
        if(S.size()>min_feature_length) {
            for(vector<int>::iterator itt = S.begin(); itt != S.end(); itt++) {
                result.push_back(edges[*itt].e);
                for(seg_edge_list::iterator ittt = edges.begin(); ittt != edges.end(); ittt++) {
                    if( edges[*itt].canLinkTo(*ittt)!=NO_LINK ) { /* neighbors and themselves */
                        ittt->islockedA = true;
                        ittt->islockedB = true;
                    }
                }
            }
        }
    }
    cout<<"split line numbers before connecting: "<<sum<<endl;
}

void segmentation(const aiMesh *mesh, edge_face_map &efm, edge_list &split_edges) {
    face_normals fn;
    gen_face_normals(mesh,fn);
    cout<<"gen_face_normals() done"<<endl;

    seg_edge_list edges;
    vector<angle_index> a_i_list;
    edge_face_map::iterator iter;
    int fA,fB;
    int index = 0;
    for(iter = efm.begin(); iter != efm.end(); iter++) {
        fA = iter->second.fA; fB = iter->second.fB;
        if(fB>=0) {
            float angle = fn[fA]*fn[fB]; /* normals in fn should have been normalized */
            edges.push_back(seg_edge(iter->first,1-angle)); /* 1-x has the same tendency as acos(x), and is >= 1*/
            //if(angle>=1) cout<<"a"<<angle<<" ";
            a_i_list.push_back(angle_index(angle,index)); 
                /* x has the opposite tendency as acos(x), so it is enough for sorting */          
        }
        else {
            edges.push_back(seg_edge(iter->first,VERY_SMALL_SHARPNESS));
        }
        index++;
    }

    sort(a_i_list.begin(),a_i_list.end());
    cout<<"sort() done"<<endl;

    int num = threshold_one * a_i_list.size();
    for(int i=0; i<num; i++) {
        edges[a_i_list[i].id].isChosen = true;
    }

    expand_features(edges,split_edges);
    cout<<"split line number: "<<split_edges.size()<<endl;
}

void scene_segmentation(const aiScene *sc, scene_edge_face_map &scene_efm, scene_edge_list &result) {
    if(result.size()>0)
        return;
    for(int m=0; m<sc->mNumMeshes; m++) {
        edge_list split_edges;
        segmentation(sc->mMeshes[m], scene_efm[m], split_edges);
        result.push_back(split_edges);
    }
}