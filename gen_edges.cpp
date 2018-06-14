#include "my_header.h"
#include <iostream>
using namespace std;

void recursive_gen_edges (const aiScene *sc, const aiNode* nd, scene_edges all_edges)
{
    unsigned int m,f,i;
    int va,vb;
    edge_face_map::iterator iter;
    
    for (m=0; m < nd->mNumMeshes; ++m) {
        //cout<<"m:"<<m<<endl;
		const aiMesh* mesh = sc->mMeshes[nd->mMeshes[m]];
		mesh_edges me;
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
				iter = me.find(edge(va,vb));
                if(iter != me.end()) {
                    iter->second.insert(f);
                    //cout<<va<<","<<vb<<" : "<<iter->second.fA<<","<<iter->second.fB<<endl;
                }
                else {
				    me.insert( edge_face_map::value_type (edge(va,vb),face_pair(f)) );
				    //cout<<va<<","<<vb<<" : "<<f<<","<<-1<<endl;
				}
		    }
		}
		all_edges.push_back(&me);
	}
	
	/* for all children */
	for (int n = 0; n < nd->mNumChildren; ++n) {
		recursive_gen_edges(sc, nd->mChildren[n], all_edges);
	}
}

void gen_edges (const aiScene *sc, scene_edges all_edges) 
{
    recursive_gen_edges(sc, sc->mRootNode, all_edges);
}

/*int main() {
    edge_face_map m;
    edge_face_map::iterator iter;
    m.insert(edge_face_map::value_type (edge(1,2),face_pair(3)) );
    m.insert(edge_face_map::value_type (edge(4,1),face_pair(4)) );
    for(iter = m.begin(); iter != m.end(); iter++) {
        std::cout<<iter->first.pA<<","<<iter->first.pB<<" : "<<iter->second.fA<<" "<<iter->second.fB<<std::endl;
        std::cout<<(edge(2,1)==iter->first)<<std::endl;
    }
    iter = m.find(edge(2,1));
    if(iter != m.end())
        std::cout<<"Find"<<std::endl;
    else
        std::cout<<"Do not Find"<<std::endl;
    return 0;
}*/
