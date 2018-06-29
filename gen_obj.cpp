#include "my_header.h"
using namespace std;

void gen_obj(const chart_list &all_charts, scene_UV_list &all_UV) {
    if(all_UV.size()==0)
        return;
    ofstream of("output.obj");

    if (of.is_open())
    {      
        vector<int> start_pos_list_long; //len is the number of charts
        vector<int> start_pos_list_short; //len is the number of meshes
        start_pos_list_long.push_back(0);
        start_pos_list_short.push_back(0);

        vector<int> mesh_ids; //len is the number of charts
        for(int c=0; c<all_charts.size(); c++)
            mesh_ids.push_back(-1);
        vector<const aiMesh*> mesh_list;
        for(int c=0; c<all_charts.size(); c++) {
            const aiMesh *mesh = all_charts[c].mi.mesh;
            vector<const aiMesh*>::const_iterator iter = find(mesh_list.begin(),mesh_list.end(),mesh);
            if(iter == mesh_list.end()) {
                mesh_ids[c] = mesh_list.size();
                mesh_list.push_back(mesh);
            }
            else {
                mesh_ids[c] = iter-mesh_list.begin();
            }
        }
        for(int m=0; m<mesh_list.size(); m++) {
            const aiMesh *mesh = mesh_list[m];
            for (int v=0; v<mesh->mNumVertices; v++) {
			    const aiVector3D *vertex = &mesh->mVertices[v];
                of<<"v "<<vertex->x<<" "<<vertex->y<<" "<<vertex->z<<endl;
            }
            start_pos_list_short.push_back(start_pos_list_short.back()+mesh->mNumVertices);
        }
        for(int m=0; m<mesh_list.size(); m++) {
            const aiMesh *mesh = mesh_list[m];
            for (int v=0; v<mesh->mNumVertices; v++) {
			    const aiVector3D *normal = &mesh->mNormals[v];
                of<<"vn "<<normal->x<<" "<<normal->y<<" "<<normal->z<<endl;
            }
        }

        for(int c=0; c<all_charts.size(); c++) {
            const aiMesh *mesh = all_charts[c].mi.mesh;
            const vector<chart_face> &faces = all_charts[c].faces;
            const vector<int> &vertices = all_charts[c].vertices;
            for(int i=0; i<vertices.size(); i++) {
                const Eigen::Vector2d *uv = &all_UV[c][i];
                of<<"vt "<<uv->x()<<" "<<uv->y()<<endl;
            }
            for(vector<chart_face>::const_iterator it=faces.begin(); it!=faces.end(); it++) {
                of<<"f ";
                for(int i=0; i<3; i++) {
                    int index_short = start_pos_list_short[mesh_ids[c]] + vertices[it->indices[i]] + 1;
                    int index_long = start_pos_list_long[c] + it->indices[i] + 1;
                    of<<index_short<<"/"<<index_long<<"/"<<index_short<<" "; /* v/vt/vn */
                }
                of<<endl;
            }
            start_pos_list_long.push_back(start_pos_list_long.back()+vertices.size());
        }   

        of<<endl; /* it seems that is is not necessary */

        of.close();
    }
    cout<<"gen_obj() done"<<endl;
}
