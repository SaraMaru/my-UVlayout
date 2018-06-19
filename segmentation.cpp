#include "my_header.h"
using namespace std;

const float threshold_one = 0.05f; /* kept threshold_one/100 of the detected edges */
const float threshold_two = 0.08f; /* will not continue connecting if weight < max_string_length * threshold_two */
const int min_feature_length = 15;
const int max_string_length = 5;
const float VERY_SMALL_SHARPNESS = -10;
const float merge_tendency = 0.25;

void find_next(seg_edge_list &edges, const vertex_edge_map &vem, bool *lock_list, const int index,
        float sharpness, float &max_sharpness, vector<int> &S, vector<int> &best_path, vector<int> &P) {

    int point;
    float prev_sharpness = sharpness;
    sharpness += edges[index].sharpness;
    S.push_back(index);
    int len = S.size();
    //cout<<index<<"*"<<len<<" ";
    if(sharpness>max_sharpness) {
        max_sharpness = sharpness;
        best_path.assign(S.begin(),S.end());
    }

    if(len<max_string_length) {
        if(len>1) {
            int i = *(S.end()-2);
            point = edges[index].adjacentTo(edges[i]);
            P.push_back(point);
            lock_list[point] = true;
        }

        vector<int>::const_iterator iter;
        int p;
        p = edges[index].e.pA;
        if(!lock_list[p]) {
            const vector<int> &vA = vem.find(p)->second;
            for(iter=vA.begin(); iter!=vA.end(); iter++)
                if(*iter!=index)
                    find_next(edges,vem,lock_list,*iter,sharpness,max_sharpness,S,best_path,P);
        }
        p = edges[index].e.pB;
        if(!lock_list[p]) {
            const vector<int> &vB = vem.find(p)->second;
            for(iter=vB.begin(); iter!=vB.end(); iter++)
                if(*iter!=index)
                    find_next(edges,vem,lock_list,*iter,sharpness,max_sharpness,S,best_path,P);
        }

        if(len>1) {
            lock_list[point] = false;
            P.pop_back();
        }
    }
   
    S.pop_back();
    sharpness = prev_sharpness;
}

void easy_find_next(seg_edge_list &edges, int index, vector<int> &result, int len, float weight, bool *lock_list) {
    float max_sharpness = VERY_SMALL_SHARPNESS;
    int new_index = -1;
    int repeat = edges.size();
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
    
    if(new_index<0) { /* can not find a new edge */
        int prev_e, point;
        for(int i=0; i<len; i++) {
            prev_e = result.back();
            result.pop_back();
            point = edges[result.back()].adjacentTo(edges[prev_e]);
            lock_list[point] = false;
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
        easy_find_next(edges,new_index,result,len,weight,lock_list);
    }
    else if(weight < max_string_length*threshold_two) {
        int prev_e, point;
        for(int i=0; i<max_string_length; i++) {
            prev_e = result.back();
            result.pop_back();
            point = edges[result.back()].adjacentTo(edges[prev_e]);
            lock_list[point] = false;
        }
    }
}

void expand_features(const mesh_info &mi, seg_edge_list &edges, edge_list &result) {
    const vertex_edge_map &vem = mi.vem;
    const int v_num = mi.mesh->mNumVertices;
    bool *is_locked = new bool[v_num];
    //cout<<"v_num:"<<v_num<<endl;
    for(int i=0; i<v_num; i++)
        is_locked[i] = false;
    int sum = 0;
    for(seg_edge_list::iterator it = edges.begin(); it != edges.end(); it++) {
        if(!it->isChosen)
            continue;
        sum++;
        vector<int> feature;
        //easy_find_next(edges, it-edges.begin(), feature, 0, 0, is_locked);

        feature.push_back(it-edges.begin());
        float max_sharpness;
        do {
            max_sharpness = 0;
            vector<int> S,best_path,P;
            find_next(edges,vem,is_locked,feature.back(),0,max_sharpness,S,best_path,P);
            if(best_path.size()>1) {
                feature.push_back(best_path[1]);
                //cout<<"{"<<best_path[0]<<" "<<best_path[1]<<"}"<<endl;
                int p = edges[best_path[0]].adjacentTo(edges[best_path[1]]);
                is_locked[p] = true;
            }
            else
                break;
        } while(max_sharpness > max_string_length*threshold_two);
        
        cout<<"_"<<feature.size()<<"_ ";
        if(feature.size()>min_feature_length) {
            for(vector<int>::iterator itt = feature.begin(); itt != feature.end(); itt++) {
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

int calc_feature_dists(const mesh_info &mi, const edge_list &split_edges, int* dists) {
    const aiMesh *mesh = mi.mesh;
    const edge_list el = mi.el;
    const raw_edge_face_map refm = mi.refm;

    set<edge> marked_edges;
    queue<int> face_queue;
    edge_list edges;
    int dist = 1;
    edges.assign(split_edges.begin(),split_edges.end());

    while(!edges.empty()) {
        for(int i=0; i<edges.size(); i++) {
            marked_edges.insert(edges[i]);
            face_pair fp = refm.find(edges[i])->second;
            if(dists[fp.fA]<0 || dists[fp.fA]>dist) { /* dists[.] is initilized to -1 */
                face_queue.push(fp.fA);
                dists[fp.fA] = dist;
            }
            if(dists[fp.fB]<0 || dists[fp.fB]>dist) {
                face_queue.push(fp.fB);
                dists[fp.fB] = dist;
            }
        }
        edges.clear();
        while(!face_queue.empty()) {
            int f = face_queue.front();
            //cout<<".."<<f<<".. ";
            face_queue.pop();
            const aiFace* face = &mesh->mFaces[f];
            for(int i = 0; i < face->mNumIndices; i++) {
                int va,vb;
                va = face->mIndices[i];
                if(i==face->mNumIndices-1)
                    vb = face->mIndices[0];
                else
                    vb = face->mIndices[i+1];
                edge e = edge(va,vb);
                set<edge>::iterator iter = marked_edges.find(e);
                if(iter==marked_edges.end()) /* e is not in the set */
                    edges.push_back(e);
            }
        }
        dist++;
    }
    return dist-1;
}

void expand_charts(const mesh_info &mi, const edge_list &split_edges) {
    const aiMesh *mesh = mi.mesh;
    const edge_list el = mi.el;
    const raw_edge_face_map refm = mi.refm;
    int f_num = mesh->mNumFaces;
    int *dists = new int[f_num];
    for(int f=0; f<f_num; f++)
        dists[f] = -1;
    int max_dist = calc_feature_dists(mi,split_edges,dists);
    float merge_dist = max_dist*merge_tendency;
    cout<<"calc_feature_dists() done"<<endl;
    cout<<"max_dist: "<<max_dist<<endl;

    vector<int> seed;
    for(int f=0; f<f_num; f++) {
        aiFace *face = &mesh->mFaces[f];
        vector<int> adj_faces;
        for(int i=0; i<face->mNumIndices; i++) {
            int va,vb;
            va = face->mIndices[i];
            if(i==face->mNumIndices-1)
                vb = face->mIndices[0];
            else
                vb = face->mIndices[i+1];
            face_pair fp = refm.find(edge(va,vb))->second;
            if(fp.fA==f)
                adj_faces.push_back(fp.fB);
            else
                adj_faces.push_back(fp.fA);
        }
        int i;
        for(i=0; i<adj_faces.size(); i++) {
            if(dists[adj_faces[i]]>=dists[f])
                break;
        }
        if(i>=adj_faces.size()) {
            seed.push_back(f);
            cout<<f<<"/"<<dists[f]<<" ";
        }
    }
    /*vector<int>::iterator it;
    for(it=seed.begin(); it!=seed.end();) {
        if(dists[*it]<max_dist/2)
            it = seed.erase(it);
        else
            it++;
    }
    for(it=seed.begin(); it!=seed.end(); it++)
        cout<<(*it)<<"//"<<dists[*it]<<" ";*/

    int *charts = new int[f_num];
    for(int f=0; f<f_num; f++)
        charts[f] = -1;
    set<edge> boundaries;
    for(int e=0; e<el.size(); e++)
        boundaries.insert(el[e]);
    cout<<"set size: "<<boundaries.size()<<endl;
    vector<dist_index> d_i_heap;
    for(vector<int>::iterator it=seed.begin(); it!=seed.end(); it++) {
        charts[*it] = *it;
        d_i_heap.push_back(dists[*it],*it);
    }
    make_heap(d_i_heap.begin(), d_i_heap.end(), less<dist_index>());

    while(!d_i_heap.empty()) {
        int face_id = d_i_heap.front().id;
        const aiFace *face = &mesh->mFaces[face_id];
        pop_heap(d_i_heap.begin(),d_i_heap.end());
        d_i_heap.pop_back();

        for(int i=0; i<face->mNumIndices; i++) {
            int va,vb;
            va = face->mIndices[i];
            if(i==face->mNumIndices-1)
                vb = face->mIndices[0];
            else
                vb = face->mIndices[i+1];
            face_pair fp = refm.find(edge(va,vb))->second;
            int opp_face_id;
            if(fp.fA==f)
                opp_face_id = fp.fB;
            else
                opp_face_id = fp.fA;
            if(charts[opp_face_id]<0) { /* if opp_face's chart is not defined */
                charts[opp_face_id] = charts[face_id];
                boundaries.erase(edge(va,vb));
                const aiFace *opp_face = &mesh->mFaces[opp_face_id];
                for(int ii=0; ii<opp_face->mNumIndices; ii++) { /* only for triangles */
                    int vc = opp_face->mIndices[ii];
                    if(vc==va || vc==vb)
                        continue;
                    boundaries.insert(edge(vc,va));
                    boundaries.insert(edge(vc,vb));
                }
            }
            else if( charts[opp_face_id] != charts[face_id] && dists[charts[face_id]]-dists[face_id] < merge_dist &&
                    dists[charts[opp_face_id]]-dists[opp_face_id] < merge_dist ) {
                int c1 = charts[face_id];  int c2 = charts[opp_face_id];
                int c = dists[c1]>=dists[c2] ? c1 : c2;
            }
        }
    }

    delete[] dists;
}

void segment(const mesh_info &mi, edge_list &split_edges) {
    const aiMesh *mesh = mi.mesh;
    const edge_face_map &efm = mi.efm;
    const edge_list &el = mi.el;
    const face_normals &fn = mi.fn;

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
            //cout<<"a"<<angle<<" ";
            a_i_list.push_back(angle_index(angle,index)); 
                /* x has the opposite tendency as acos(x), so it is enough for sorting */          
        }
        else {
            edges.push_back(seg_edge(el[index],VERY_SMALL_SHARPNESS));
        }
    }

    sort(a_i_list.begin(),a_i_list.end());

    int num = threshold_one * a_i_list.size();
    for(int i=0; i<num; i++) {
        edges[a_i_list[i].id].isChosen = true;
        //split_edges.push_back( edges[a_i_list[i].id].e);
    }

    expand_features(mi,edges,split_edges);
    cout<<"split line number: "<<split_edges.size()<<endl;

    expand_charts(mi,split_edges);
    cout<<"expand_charts() done"<<endl;
}

void scene_segment (const scene_info &si, scene_edge_list &result) {
    if(result.size()>0)
        return;
    for(int m=0; m<si.sc->mNumMeshes; m++) {
        edge_list split_edges;
        mesh_info mi = mesh_info(si.sc->mMeshes[m], si.s_el[m], si.s_efm[m], si.s_refm[m], si.s_vem[m], si.s_fn[m]);
        segment(mi, split_edges);
        result.push_back(split_edges);
    }
}