#include "my_header.h"
#include <iostream>
using namespace std;

void gen_edges (const aiMesh* mesh, edge_face_map &efm)
{
    unsigned int m,f,i;
    int va,vb;
    edge_face_map::iterator iter;    

    for (f = 0; f < mesh->mNumFaces; ++f) {
        const aiFace* face = &mesh->mFaces[f];
        //cout<<"f:"<<f<<"<"<<face->mNumIndices<<">"<<endl;
        for(i = 0; i < face->mNumIndices; i++) {
            //cout<<"i:"<<i<<endl;
            va = face->mIndices[i];
            if(i==face->mNumIndices-1)
                vb = face->mIndices[0];
            else
                vb = face->mIndices[i+1];
            iter = efm.find(edge(va,vb));
            if(iter != efm.end()) {
                iter->second.insert(f);
                //cout<<va<<","<<vb<<" : "<<iter->second.fA<<","<<iter->second.fB<<endl;
            }
            else {
                efm.insert( edge_face_map::value_type (edge(va,vb),face_pair(f)) );
                //cout<<va<<","<<vb<<" : "<<f<<","<<-1<<endl;
            }
        }
    }
}
