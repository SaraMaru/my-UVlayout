#include "my_header.h"
#include <iostream>
using namespace std;

void gen_edges (const aiMesh *mesh, edge_list &el)
{
    unsigned int f,i;
    int va,vb;

    for (f = 0; f < mesh->mNumFaces; ++f) {
        const aiFace *face = &mesh->mFaces[f];
        for(i = 0; i < face->mNumIndices; i++) {
            va = face->mIndices[i];
            if(i==face->mNumIndices-1)
                vb = face->mIndices[0];
            else
                vb = face->mIndices[i+1];
            el.push_back(edge(va,vb));
        }
    }

    sort(el.begin(), el.end());  
    el.erase( unique(el.begin(), el.end()), el.end() ); /* erase duplicated elements */
}

void gen_edge_face_map (const aiMesh *mesh, const edge_list &el, edge_face_map &efm)
{
    unsigned int f,i;
    int va,vb;
    map<edge,face_pair> raw_efm;
    map<edge,face_pair>::iterator iter;    

    for (f = 0; f < mesh->mNumFaces; ++f) {
        const aiFace* face = &mesh->mFaces[f];
        for(i = 0; i < face->mNumIndices; i++) {
            va = face->mIndices[i];
            if(i==face->mNumIndices-1)
                vb = face->mIndices[0];
            else
                vb = face->mIndices[i+1];
            iter = raw_efm.find(edge(va,vb));
            if(iter != raw_efm.end()) {
                iter->second.insert(f);
            }
            else {
                raw_efm.insert( map<edge,face_pair>::value_type(edge(va,vb),face_pair(f)) );
            }
        }
    }

    for(int index=0; index<el.size(); index++) {
        iter = raw_efm.find(el[index]);
        efm.insert( edge_face_map::value_type(index,iter->second) );
    }
}

void gen_vertex_edge_map (const edge_list &el, vertex_edge_map &vem)
{
    int len = el.size();
    for(int i=0; i<len; i++) {
        int a = el[i].pA; int b = el[i].pB;
        vertex_edge_map::iterator vem_iter;
        vem_iter = vem.find(a);
        if(vem_iter == vem.end()) {
            vector<int> v;
            v.push_back(i);
            vem.insert( vertex_edge_map::value_type(a,v) );
        }
        else
            vem_iter->second.push_back(i);
        vem_iter = vem.find(b);
        if(vem_iter == vem.end()) {
            vector<int> v;
            v.push_back(i);
            vem.insert( vertex_edge_map::value_type(b,v) );
        }
        else
            vem_iter->second.push_back(i);
    }
}