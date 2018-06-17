#include "my_header.h"
#include <iostream>
using namespace std;

const float threshold_one = 0.05f; /* kept threshold_one/100 of the detected edges */
const float threshold_two = 0.02f; /* will not continue connecting if weight < max_string_length * threshold_two */
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

void connect_to_others(seg_edge_list &edges, int index, vector<int> &result, int len, float weight, bool *lock_list) {
    float max_sharpness = VERY_SMALL_SHARPNESS;
    int new_index = -1;
    int repeat = edges.size();
    //cout<<"repeat:"<<repeat<<endl;
    for(int i=0; i<repeat; i++) {
        int p = edges[index].adjacentTo(edges[i]);
        if(p>=0 && !lock_list[p]) { /* neighbors */
            float s = edges[i].sharpness;
            if(s>max_sharpness) {
                max_sharpness = s;
                new_index = i;
            }
        }
    }
    //cout<<"s"<<max_sharpness<<" ";
    
    if(new_index<0) { /* can not find a new edge */
        for(int i=0; i<len; i++) {
            /*if(result.back()>=35947 || result.back()<0) 
                cout<<"result:"<<result.back()<<endl;*/
            //lock_list[result.back()] = false;
            result.pop_back();
        }
        return;
    }

    /* link a and new_index */
    result.push_back(new_index);
    int point = edges[index].adjacentTo(edges[new_index]);
    lock_list[point] = true;

    if(edges[index].isChosen) {
        weight = 0;
        len = 0;
    }
    else {
        weight += max_sharpness;
        len++;
    }
    if(len<max_string_length) {
        connect_to_others(edges,new_index,result,len,weight,lock_list);
    }
    else if(weight < max_string_length*threshold_two) {
        for(int i=0; i<max_string_length; i++) {
            /*if(result.back()>=35947 || result.back()<0) 
                cout<<"result:"<<result.back()<<endl;*/
            //lock_list[result.back()] = false;
            result.pop_back();
        }
    }
}

void expand_features(const mesh_info &mi, seg_edge_list &edges, edge_list &result) {
    const int v_num = mi.mesh->mNumVertices;
    bool *is_locked = new bool[v_num];
    cout<<"v_num:"<<v_num<<endl;
    for(int i=0; i<v_num; i++)
        is_locked[i] = false;
    int sum = 0;
    for(seg_edge_list::iterator it = edges.begin(); it != edges.end(); it++) {
        if(!it->isChosen)
            continue;
        sum++;
        vector<int> S;
        connect_to_others(edges, it-edges.begin(), S, 0, 0, is_locked);
        cout<<S.size()<<" ";
        if(S.size()>min_feature_length) {
            for(vector<int>::iterator itt = S.begin(); itt != S.end(); itt++) {
                result.push_back(edges[*itt].e);
                for(seg_edge_list::iterator ittt = edges.begin(); ittt != edges.end(); ittt++) {
                    if( edges[*itt].adjacentTo(*ittt)>=0 ) { /* neighbors */
                        is_locked[ittt->e.pA] = true;
                        is_locked[ittt->e.pB] = true;
                        ittt->isChosen = false;
                    }
                }
            }
        }
    }
    delete[] is_locked;
    cout<<"split line numbers before connecting: "<<sum<<endl;
}

void segmentation(const mesh_info &mi, edge_list &split_edges) {
    const aiMesh *mesh = mi.mesh;
    const edge_face_map &efm = mi.efm;
    const edge_list &el = mi.el;

    face_normals fn;
    gen_face_normals(mesh,fn);
    cout<<"gen_face_normals() done"<<endl;

    seg_edge_list edges;
    vector<angle_index> a_i_list;
    int index;
    int fA,fB;
    for(index=0; index<el.size(); index++) {
        edge_face_map::const_iterator efm_iter = efm.find(index);
        fA = efm_iter->second.fA;  fB = efm_iter->second.fB;
        if(fB>=0) {
            float angle = fn[fA]*fn[fB]; /* normals in fn should have been normalized */
            edges.push_back(seg_edge(el[index],1-angle)); /* 1-x has the same tendency as acos(x), and is >= 1*/
            //if(angle>=1) cout<<"a"<<angle<<" ";
            a_i_list.push_back(angle_index(angle,index)); 
                /* x has the opposite tendency as acos(x), so it is enough for sorting */          
        }
        else {
            edges.push_back(seg_edge(el[index],VERY_SMALL_SHARPNESS));
        }
    }
    cout<<"efm_size:"<<efm.size()<<" "<<"edges_size:"<<edges.size()<<endl;

    sort(a_i_list.begin(),a_i_list.end());
    cout<<"sort() done"<<endl;

    int num = threshold_one * a_i_list.size();
    for(int i=0; i<num; i++) {
        edges[a_i_list[i].id].isChosen = true;
        //split_edges.push_back( edges[a_i_list[i].id].e);
    }

    expand_features(mi,edges,split_edges);

    cout<<"split line number: "<<split_edges.size()<<endl;
}

void scene_segmentation (const aiScene *sc, const scene_edge_list &scene_el, const scene_edge_face_map &scene_efm,
        const scene_vertex_edge_map &scene_vem, scene_edge_list &result) {
    if(result.size()>0)
        return;
    for(int m=0; m<sc->mNumMeshes; m++) {
        edge_list split_edges;
        mesh_info mi = mesh_info(sc->mMeshes[m], scene_el[m], scene_efm[m], scene_vem[m]);
        segmentation(mi, split_edges);
        result.push_back(split_edges);
    }
}