#include "my_header.h"
using namespace std;

void gen_obj(const scene_info &si, scene_UV_list &all_UV) {
    if(all_UV.size()<=0)
        return;
    const aiScene* sc = si.sc;
    ofstream of("output.obj");
    if (of.is_open())
    {      
        for(unsigned int m=0; m<sc->mNumMeshes; m++) {
            const aiMesh *mesh = sc->mMeshes[m];
            of<<"g default"<<endl;
            for(int v=0; v<mesh->mNumVertices; v++) {
                const aiVector3D *vertex = &mesh->mVertices[v];
                of<<"v "<<vertex->x<<" "<<vertex->y<<" "<<vertex->z<<endl;
            }
            for(int v=0; v<mesh->mNumVertices; v++) {
                const aiVector2D *uv = &all_UV[m][v];
                of<<"vt "<<uv->x<<" "<<uv->y<<endl;
            }
            for(int v=0; v<mesh->mNumVertices; v++) {
                const aiVector3D *normal = &mesh->mNormals[v];
                of<<"vn "<<normal->x<<" "<<normal->y<<" "<<normal->z<<endl;
            }
            of<<"s "<<m+1<<endl;
            of<<"g group"<<m+1<<endl;
            for(int f=0; f<mesh->mNumFaces; f++) {
                const aiFace *face = &mesh->mFaces[f];
                of<<"f ";
                for(int i=0; i<face->mNumIndices; i++) {
                    int index = face->mIndices[i]+1;
                    of<<index<<"/"<<index<<"/"<<index<<" ";
                }
                of<<endl;
            }
        }
        of<<endl; /* do not forget */
        of.close();
    }
    cout<<"gen_obj() done"<<endl;
}