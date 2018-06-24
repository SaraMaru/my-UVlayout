#include "my_header.h"
#include "OpenNL_psm.h"
using namespace Eigen;

#define VERY_BIG_NUMBER 999999

Vector3d trans(const aiVector3D &v) {
    Vector3d vv(v.x,v.y,v.z);
    return vv;
}

double dist(const Vector3d &a, const Vector3d &b) {
    return (a-b).norm();
}

double dist(aiVector3D &a, aiVector3D &b) {
    //return dist(trans(a),trans(b));
    return (a-b).Length();
}

/* each triangle is provided with a local orthonormal basis */
void get_coordinates(const chart &ch, triangle *ts) {
    const aiMesh *mesh = ch.mi.mesh;
    face_normals fn = ch.mi.fn;
    int index = 0;
    for(int f=0; f<mesh->mNumFaces; f++) {
        if(ch.faces.find(f)==ch.faces.end())
            continue;
        const aiFace *face = &mesh->mFaces[f];
        for(int i=0; i<3; i++) {
            Vector3d v1 = trans(mesh->mVertices[face->mIndices[i]]);
            Vector3d v2 = trans(mesh->mVertices[face->mIndices[(i+1)%3]]);
            Vector3d v3 = trans(mesh->mVertices[face->mIndices[(i+2)%3]]);
            Vector3d x_axis = v2-v1;
            x_axis.normalize();
            Vector3d y_axis = trans(fn[index]).cross(x_axis);
            y_axis.normalize(); //actually not needed
            Vector3d v1v3 = v3-v1;
            float x = v1v3.dot(x_axis);
            float y = v1v3.dot(y_axis);
            ts[index][(i+2)%3] = Vector2d(x,y);
        }
        std::cout<<"f"<<index<<": ("<<ts[index][0].transpose()<<") ("<<ts[index][1].transpose()<<") ("
            <<ts[index][2].transpose()<<")"<<std::endl;
        std::cout<<"fn: "<<trans(fn[index]).transpose()<<std::endl;
        index++;
    }
}

void old_get_coordinates(const chart &ch, triangle *ts) {
    const aiMesh *mesh = ch.mi.mesh;
    face_normals fn = ch.mi.fn;
    int index = 0;
    for(int f=0; f<mesh->mNumFaces; f++) {
        if(ch.faces.find(f)==ch.faces.end())
            continue;
        const aiFace *face = &mesh->mFaces[f];
        Vector3d v1 = trans(mesh->mVertices[face->mIndices[0]]);
        Vector3d v2 = trans(mesh->mVertices[face->mIndices[1]]);
        Vector3d v3 = trans(mesh->mVertices[face->mIndices[2]]);
        ts[index][0][0] = ts[index][0][1] = ts[index][1][1] = 0;
        ts[index][1][0] = dist(v1,v2);
        Vector3d x_axis = (v2-v1) / ts[index][1][0];
        Vector3d v1v3 = v3-v1;
        ts[index][2][0] = v1v3.dot(x_axis);
        Vector3d y_axis = trans(fn[index]).cross(x_axis);
        y_axis.normalize(); //actually not needed
        ts[index][2][1] = v1v3.dot(y_axis);
        std::cout<<"f"<<index<<": ("<<ts[index][0][0]<<","<<ts[index][0][1]<<") ("<<ts[index][1][0]<<","<<ts[index][1][1]<<") ("<<ts[index][2][0]<<","<<ts[index][2][1]<<")"<<std::endl;
        std::cout<<"fn: "<<trans(fn[index]).transpose()<<std::endl;
        index++;
    }
}

double find_pinned_vertices(const chart &ch, int &a, int &b) {
    const aiMesh *mesh = ch.mi.mesh;
    int max_x,max_y,max_z;
    int min_x,min_y,min_z;
    int max_x_id,max_y_id,max_z_id,min_x_id,min_y_id,min_z_id;
    max_x = max_y = max_z = -VERY_BIG_NUMBER;
    min_x = min_y = min_z = VERY_BIG_NUMBER;
    for(map<int,int>::const_iterator iter=ch.m_2_u.begin(); iter!=ch.m_2_u.end(); iter++) {
        //cout<<iter->first<<"%%"<<iter->second<<endl;
        int i = iter->first;
        aiVector3D v = mesh->mVertices[i];
        //cout<<v.x<<" "<<v.y<<" "<<v.z<<endl;
        if(v.x>max_x) { max_x = v.x; max_x_id = i; }
        if(v.x<min_x) { min_x = v.x; min_x_id = i; }
        if(v.y>max_y) { max_y = v.y; max_y_id = i; }
        if(v.y<min_y) { min_y = v.y; min_y_id = i; }
        if(v.z>max_z) { max_z = v.z; max_z_id = i; }
        if(v.z<min_z) { min_z = v.z; min_z_id = i; }
    }
    double dist_x = dist(mesh->mVertices[min_x_id],mesh->mVertices[max_x_id]);
    double dist_y = dist(mesh->mVertices[min_y_id],mesh->mVertices[max_y_id]);
    double dist_z = dist(mesh->mVertices[min_z_id],mesh->mVertices[max_z_id]);
    double max_dist;
    if(dist_x>dist_z) {
        if(dist_x>dist_y) { //dist_x is the biggest
            a = min_x_id;  b = max_x_id; max_dist = dist_x;
        }
        else { //dist_y is the biggest
            a = min_y_id;  b = max_y_id; max_dist = dist_y;
        }
    }
    else { 
        if(dist_y>dist_z) { //dist_y is the biggest
            a = min_y_id;  b = max_y_id; max_dist = dist_y;
        }
        else { //dist_z is the biggest
            a = min_z_id;  b = max_z_id; max_dist = dist_z;
        }
    }
    std::cout<<"NO."<<a<<": "<<trans(mesh->mVertices[a]).transpose()<<endl;
    std::cout<<"NO."<<b<<": "<<trans(mesh->mVertices[b]).transpose()<<endl;
    a = ch.m_2_u.find(a)->second;
    b = ch.m_2_u.find(b)->second;
    if(a>b) {
        int tmp = b;
        b = a;
        a = tmp;
    }
    std::cout<<"new id: "<<a<<" and "<<b<<endl;
    return max_dist;
}

void setup_conformal_map_relations(triangle &t, NLuint v0, NLuint v1, NLuint v2) {
    Vector2d z0,z1,z2;
    z0 = t[0]; z1 = t[1]; z2 = t[2];
    Vector2d z01 = z1 - z0;
    Vector2d z02 = z2 - z0;
    double a = z01[0];
    double b = z01[1];
    double c = z02[0];
    double d = z02[1];
    assert(b == 0.0);

    // Note  : 2*id + 0 --> u
    //         2*id + 1 --> v
    NLuint u0_id = 2*v0    ;
    NLuint v0_id = 2*v0 + 1;
    NLuint u1_id = 2*v1    ;
    NLuint v1_id = 2*v1 + 1;
    NLuint u2_id = 2*v2    ;
    NLuint v2_id = 2*v2 + 1;
    
    // Note : b = 0

    // Real part
    nlBegin(NL_ROW);
    nlCoefficient(u0_id, -a+c) ;
    nlCoefficient(v0_id,  b-d) ;
    nlCoefficient(u1_id,   -c) ;
    nlCoefficient(v1_id,    d) ;
    nlCoefficient(u2_id,    a);
    nlEnd(NL_ROW);

    // Imaginary part
    nlBegin(NL_ROW);
    nlCoefficient(u0_id, -b+d);
    nlCoefficient(v0_id, -a+c);
    nlCoefficient(u1_id,   -d);
    nlCoefficient(v1_id,   -c);
    nlCoefficient(v2_id,    a);
    nlEnd(NL_ROW);
}

void apply(const chart &ch, triangle *ts, int id1, int id2, UV_list &UV) {
    const int nb_eigens = 10;
    const aiMesh *mesh = ch.mi.mesh;
	nlNewContext();
    NLuint nb_vertices = NLuint(ch.m_2_u.size());

    nlSolverParameteri(NL_NB_VARIABLES, NLint(2*nb_vertices));
    nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE);
    nlSolverParameteri(NL_MAX_ITERATIONS, NLint(5*nb_vertices));
	nlSolverParameterd(NL_THRESHOLD, 1e-6);
    nlBegin(NL_SYSTEM);
    int f=0;
    for(set<int>::const_iterator it=ch.faces.begin(); it!=ch.faces.end(); it++) {
        const aiFace *face = &mesh->mFaces[*it];
        for(int ii=0;ii<3;ii++) {
            int mi = face->mIndices[ii];
            int i = ch.m_2_u.find(mi)->second;
            double u = mesh->mVertices[mi][0];
            double v = mesh->mVertices[mi][1];
            nlSetVariable(2 * i    , u);
            nlSetVariable(2 * i + 1, v);
            if(i==id1 || i==id2) {
                nlLockVariable(2 * i    );
                nlLockVariable(2 * i + 1);
            }
        }
    }
    nlBegin(NL_MATRIX);
    f=0;
    for(set<int>::const_iterator it=ch.faces.begin(); it!=ch.faces.end(); it++) {
        const aiFace *face = &mesh->mFaces[*it];
        NLuint v0 = ch.m_2_u.find(face->mIndices[0])->second;
        NLuint v1 = ch.m_2_u.find(face->mIndices[1])->second;
        NLuint v2 = ch.m_2_u.find(face->mIndices[2])->second;
        setup_conformal_map_relations(ts[f],v0,v1,v2);
        f++;
	}
    nlEnd(NL_MATRIX);
    nlEnd(NL_SYSTEM);
    std::cout << "Solving ..." << std::endl;
    nlSolve();

    UV = new aiVector2D[nb_vertices];
    for(NLuint i=0; i<nb_vertices; ++i) {
        double u = nlGetVariable(2 * i    );
        double v = nlGetVariable(2 * i + 1);
        UV[i] = aiVector2D(u,v);
    }

    double time;
    NLint iterations;	    
    nlGetDoublev(NL_ELAPSED_TIME, &time);
    nlGetIntegerv(NL_USED_ITERATIONS, &iterations);
    std::cout << "Solver time: " << time << std::endl;
    std::cout << "Used iterations: " << iterations << std::endl;
    nlDeleteContext(nlGetCurrent());
}

void OpenNL_parameterize(const chart &ch, UV_list &UV) {
    const aiMesh *mesh = ch.mi.mesh;
    int f_num = ch.faces.size();
    int v_num = ch.m_2_u.size();
    cout<<"f_num: "<<f_num<<" "<<"v_num: "<<v_num<<endl;

    triangle *ts = new triangle[f_num];
    old_get_coordinates(ch,ts);
    std::cout<<"get_coordinates() done"<<std::endl;

    int id1,id2;
    double max_dist = find_pinned_vertices(ch, id1, id2);
    std::cout<<"find_pinned_vertices() done"<<std::endl;
    std::cout<<"max_dist: "<<max_dist<<endl;

    apply(ch,ts,id1,id2,UV);

    float u_min=VERY_BIG_NUMBER, v_min=VERY_BIG_NUMBER, u_max=-VERY_BIG_NUMBER, v_max=-VERY_BIG_NUMBER;
    for(int i=0; i<v_num; i++) {
	    u_min = std::min(u_min, UV[i][0]);
	    u_max = std::max(u_max, UV[i][0]);
	    v_min = std::min(v_min, UV[i][1]);
	    v_max = std::max(v_max, UV[i][1]);	    
	}
	float len = std::max(u_max-u_min,v_max-v_min);
    for(int i=0; i<v_num; i++) {
	    UV[i][0] = (UV[i][0]-u_min)/len*max_dist;
	    UV[i][1] = (UV[i][1]-v_min)/len*max_dist;
	}
}

void OpenNL_scene_parameterize (const scene_info &si, chart_list &all_charts, scene_UV_list &SUVL) {
    if(SUVL.size()>0 || all_charts.size()==0)
        return;
    for(chart_list::const_iterator iter=all_charts.begin(); iter!=all_charts.end(); iter++) {
        cout<<"chart "<<iter-all_charts.begin()<<" :"<<endl;
        UV_list UV;
        OpenNL_parameterize(*iter,UV);
        SUVL.push_back(UV);
    }
}