#include "my_header.h"
using namespace std;

void gen_obj(const chart_list &all_charts, scene_UV_list &all_UV) {
    if(all_UV.size()<=0)
        return;
    ofstream of("output.obj");

    if (of.is_open())
    {      
        vector<int> start_pos_list;
        start_pos_list.push_back(0);

        for(int c=0; c<all_charts.size(); c++) {
            of<<"g chart"<<c+1<<endl;
            const aiMesh *mesh = all_charts[c].mi.mesh;
            const set<int> &faces = all_charts[c].faces;
            const map<int,int> &m2u = all_charts[c].m_2_u;

            for(map<int,int>::const_iterator iter=m2u.begin(); iter!=m2u.end(); iter++) {
                const aiVector3D *vertex = &mesh->mVertices[iter->first];
                of<<"v "<<vertex->x<<" "<<vertex->y<<" "<<vertex->z<<endl;
            }
            for(int i=0; i<m2u.size(); i++) {
                const aiVector2D *uv = &all_UV[c][i];
                of<<"vt "<<uv->x<<" "<<uv->y<<endl;
            }
            for(map<int,int>::const_iterator iter=m2u.begin(); iter!=m2u.end(); iter++) {
                const aiVector3D *normal = &mesh->mNormals[iter->first];
                of<<"vn "<<normal->x<<" "<<normal->y<<" "<<normal->z<<endl;
            }
            for(set<int>::iterator it=faces.begin(); it!=faces.end(); it++) {
                const aiFace *face = &mesh->mFaces[*it];
                of<<"f ";
                for(int i=0; i<face->mNumIndices; i++) {
                    int index = start_pos_list[c]+face->mIndices[i]+1;
                    of<<index<<"/"<<index<<"/"<<index<<" ";
                }
                of<<endl;
            }
            start_pos_list.push_back(start_pos_list.back()+m2u.size());
        }   

        of<<endl; /* it seems that is is not necessary */

        of.close();
    }
    cout<<"gen_obj() done"<<endl;
}

void old_gen_obj(const chart_list &all_charts, scene_UV_list &all_UV) {
    if(all_UV.size()<=0)
        return;
    ofstream of("output.obj");

    if (of.is_open())
    {      
        vector<int> start_pos_list;
        start_pos_list.push_back(0);
        //of<<"g default"<<endl;

        for(int c=0; c<all_charts.size(); c++) {
            of<<"g chart_"<<c+1<<"_vertices"<<endl;
            const aiMesh *mesh = all_charts[c].mi.mesh;
            vector<int> vs;
            for(int v=0; v<mesh->mNumVertices; v++) {
                if(all_charts[c].m_2_u.find(v)!=all_charts[c].m_2_u.end()) {
                    vs.push_back(v);
                    const aiVector3D *vertex = &mesh->mVertices[v];
                    of<<"v "<<vertex->x<<" "<<vertex->y<<" "<<vertex->z<<endl;
                }
            }
            for(int i=0; i<vs.size(); i++) {
                const aiVector2D *uv = &all_UV[c][i];
                of<<"vt "<<uv->x<<" "<<uv->y<<endl;
            }
            for(int i=0; i<vs.size(); i++) {
                const aiVector3D *normal = &mesh->mNormals[vs[i]];
                of<<"vn "<<normal->x<<" "<<normal->y<<" "<<normal->z<<endl;
            }
            start_pos_list.push_back(start_pos_list.back()+vs.size());
        }

        for(int c=0; c<all_charts.size(); c++) {
            of<<"g chart_"<<c+1<<"_faces"<<endl;
            const aiMesh *mesh = all_charts[c].mi.mesh;
            const set<int> &faces = all_charts[c].faces;
            for(set<int>::iterator it=faces.begin(); it!=faces.end(); it++) {
                const aiFace *face = &mesh->mFaces[*it];
                of<<"f ";
                for(int i=0; i<face->mNumIndices; i++) {
                    int index = start_pos_list[c]+face->mIndices[i]+1;
                    of<<index<<"/"<<index<<"/"<<index<<" ";
                }
                of<<endl;
            }
        }

        of<<endl; /* it seems that is is not necessary */

        of.close();
    }
    cout<<"gen_obj() done"<<endl;
}