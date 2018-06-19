#ifndef MY_HEADER_H
#define MY_HEADER_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <algorithm> /* for heap */
#include <GL/glut.h>
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <FreeImage.h> /* for grab() */
using namespace std;

/* -------------------------------------------------------------------------------- */

/*enum link_type {
    NO_LINK, LINK_AA, LINK_AB, LINK_BA, LINK_BB
};*/

struct edge {
    int pA,pB;
    edge(int a, int b) { /* important for find() and < */
        if(a<b) {
            pA = a; pB =b;
        }
        else {
            pA = b; pB = a;
        }
    }
    friend bool operator == (const edge& eA, const edge& eB) {
        if(eA.pA==eB.pA && eA.pB==eB.pB)
            return true;
        return false;
    }
    friend bool operator != (const edge& eA, const edge& eB) {
        return !(eA==eB);
    }
    friend bool operator < (const edge& eA, const edge& eB) {
        if(eA.pA<eB.pA)
            return true;
        else if(eA.pA==eB.pA && eA.pB<eB.pB)
            return true;
        return false;
    }
};

struct seg_edge {
    edge e;
    float sharpness; //angle
    bool isChosen;
    //bool islockedA;
    //bool islockedB;

    seg_edge(edge ee) : e(ee) {
        isChosen = false;
        //islockedA = islockedB = false;
    }
    seg_edge(edge ee, float s) : e(ee) {
        sharpness = s;
        isChosen = false;
        //islockedA = islockedB = false;
    }
    int adjacentTo(const seg_edge &se) {
        if(e.pA==se.e.pA || e.pA==se.e.pB) {
            if(e!=se.e)
                return e.pA;
        }
        else if(e.pB==se.e.pA || e.pB==se.e.pB) {
            if(e!=se.e)
                return e.pB;
        }
        return -1;
    }
    /*link_type canLinkTo(const seg_edge &se) {
        if(se.islockedA || se.islockedB) //notice the || here (not &&)
            return NO_LINK;
        if(islockedA && islockedB)
            return NO_LINK;
        if(!se.islockedA) {
            if(e.pA==se.e.pA && !islockedA)
                return LINK_AA;
            else if(e.pB==se.e.pA && !islockedB)
                return LINK_BA;
        }
        if(!se.islockedB) {
            if(e.pA==se.e.pB && !islockedA)
                return LINK_AB;
            else if(e.pB==se.e.pB && !islockedB)
                return LINK_BB;
        }
        return NO_LINK;
    }*/
};

struct face_pair {
    int fA,fB;
    face_pair(int a, int b) : fA(a), fB(b) {}
    face_pair(int a) : fA(a), fB(-1) {}
    void insert(int b) {
        if(fB!=-1) {
            printf("ERROR:Only 2 faces can be adjacent to one edge!\n");
        }
        else {
            fB = b;
        } 
    }
};

struct angle_index {
    float angle;
    int id;
    angle_index(float a, int i) : angle(a), id(i) {}
    friend bool operator < (const angle_index& x, const angle_index& y) {
        if(x.angle<y.angle)
            return true;
        return false;
    }
};

struct triangle {
    float x1=0, y1=0;
    float x2, y2=0;
    float x3, y3;
};

/* -------------------------------------------------------------------------------- */

typedef vector<edge> edge_list; /* edges of a mesh */
typedef vector<seg_edge> seg_edge_list;
typedef vector<edge_list> scene_edge_list; /* edges of the whole scene */

typedef map<int,face_pair> edge_face_map;
typedef vector<edge_face_map> scene_edge_face_map;

typedef map<edge,face_pair> raw_edge_face_map;
typedef vector<raw_edge_face_map> scene_raw_edge_face_map;

typedef map<int,vector<int>> vertex_edge_map;
typedef vector<vertex_edge_map> scene_vertex_edge_map;

typedef aiVector3D* face_normals;
typedef vector<face_normals> scene_face_normals;

typedef aiVector2D* UV_list;
typedef vector<UV_list> scene_UV_list;

/* -------------------------------------------------------------------------------- */

struct mesh_info {
    const aiMesh *mesh;
    const edge_list &el;
    const edge_face_map &efm;
    const raw_edge_face_map &refm;
    const vertex_edge_map &vem;
    const face_normals fn;
    mesh_info(const aiMesh *mesh, const edge_list &el, const edge_face_map &efm, const raw_edge_face_map &refm,
        const vertex_edge_map &vem, const face_normals fn) : 
        mesh(mesh), el(el), efm(efm), refm(refm), vem(vem), fn(fn) {}
};

struct scene_info {
    const aiScene *sc;
    const scene_edge_list &s_el;
    const scene_edge_face_map &s_efm;
    const scene_raw_edge_face_map &s_refm; 
    const scene_vertex_edge_map &s_vem; 
    const scene_face_normals &s_fn;
    scene_info(const aiScene *sc, const scene_edge_list &s_el, const scene_edge_face_map &s_efm,
        const scene_raw_edge_face_map &s_refm, const scene_vertex_edge_map &s_vem, const scene_face_normals s_fn) :
        sc(sc), s_el(s_el), s_efm(s_efm), s_refm(s_refm), s_vem(s_vem), s_fn(s_fn) {}
};

/* -------------------------------------------------------------------------------- */

extern void gen_edges (const aiMesh *mesh, edge_list &el);
extern void gen_edge_face_map (const aiMesh *mesh, const edge_list &el, edge_face_map &efm, raw_edge_face_map &raw_efm);
extern void gen_vertex_edge_map (const edge_list &el, vertex_edge_map &vem);
extern void gen_face_normals(const aiMesh *mesh, face_normals &fn);
extern void scene_segment (const scene_info &si, scene_edge_list &result);
extern void scene_parameterize (const scene_info &si, scene_UV_list &SUVL);

/* -------------------------------------------------------------------------------- */

#endif

