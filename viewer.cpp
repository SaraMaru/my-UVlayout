/* ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
//
// If you intend to _use_ this code sample in your app, do yourself a favour
// and replace immediate mode calls with VBOs ...
//
// The vc8 solution links against assimp-release-dll_win32 - be sure to
// have this configuration built.
// ----------------------------------------------------------------------------
*/

#include "my_header.h"
using namespace std;

const aiScene* scene = NULL;
GLuint scene_list = 0;
aiVector3D scene_min, scene_max, scene_center;

scene_edge_list all_edges;
scene_edge_face_map all_efm;
scene_raw_edge_face_map all_refm;
scene_vertex_edge_map all_vem;
scene_face_normals all_fn;
//scene_info si = scene_info(scene,all_edges,all_efm,all_refm,all_vem,all_fn);

scene_edge_list all_split_edges;
chart_list all_charts;
scene_UV_list all_UV;
scene_UV_list packed_all_UV;

float eye[] = { 0.f, 0.f, 3.f };
float center[] = { 0.f, 0.f, -5.f };
float UV_eye[] = { 0.5f, 0.5f, 1.5f };
float UV_center[] = { 0.5f, 0.5f, -2.f, };

/* current rotation angle */
static float angle = 0.0f;

/* when the previous remote happened */
static GLint prev_time = 0;

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

#define WINDOW_WIDTH 900
#define WINDOW_HEIGHT 600

bool b_line_mode = false;
bool b_chart_mode = false;
bool b_UV_mode = false;
bool b_rotate = false;

/* ---------------------------------------------------------------------------- */
void reshape(int width, int height)
{
	const double aspectRatio = (float) width / height, fieldOfView = 45.0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, aspectRatio,
		0.1, 1000.0);  /* Znear and Zfar */
	glViewport(0, 0, width, height);
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box_for_node (const aiNode* nd,
    aiVector3D* min, aiVector3D* max, aiMatrix4x4* trafo
){
	aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo,&nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n) {
		const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t) {

			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp,trafo);

			min->x = aisgl_min(min->x,tmp.x);
			min->y = aisgl_min(min->y,tmp.y);
			min->z = aisgl_min(min->z,tmp.z);

			max->x = aisgl_max(max->x,tmp.x);
			max->y = aisgl_max(max->y,tmp.y);
			max->z = aisgl_max(max->z,tmp.z);
		}
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		get_bounding_box_for_node(nd->mChildren[n],min,max,trafo);
	}
	*trafo = prev;
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box (aiVector3D* min, aiVector3D* max)
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	min->x = min->y = min->z =  1e10f;
	max->x = max->y = max->z = -1e10f;
	get_bounding_box_for_node(scene->mRootNode,min,max,&trafo);
}

/* ---------------------------------------------------------------------------- */
void color4_to_float4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

/* ---------------------------------------------------------------------------- */
void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

/* ---------------------------------------------------------------------------- */
void apply_material(const aiMaterial *mtl)
{
	float c[4];

	GLenum fill_mode;
	int ret1, ret2;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	ai_real shininess, strength;
	int two_sided;
	int wireframe;
	unsigned int max;

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
		color4_to_float4(&diffuse, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
		color4_to_float4(&specular, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

	set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
		color4_to_float4(&ambient, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
		color4_to_float4(&emission, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if(ret1 == AI_SUCCESS) {
    	max = 1;
    	ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if(ret2 == AI_SUCCESS)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
        else
        	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
	else {
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
		set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	}

	if(b_line_mode) {
    	fill_mode = GL_LINE;
	}
	else {
		max = 1;
		if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
			fill_mode = wireframe ? GL_LINE : GL_FILL;
		else
			fill_mode = GL_FILL;
	}
	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		glDisable(GL_CULL_FACE);
	else
		glEnable(GL_CULL_FACE);
}

/* ---------------------------------------------------------------------------- */
void recursive_render (const aiScene *sc, const aiNode* nd)
{
	unsigned int i;
	unsigned int n = 0, t;
	aiMatrix4x4 m = nd->mTransformation;

	/* update transform */
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);

	/* draw all meshes assigned to this node */
	for (; n < nd->mNumMeshes; ++n) {
		const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];

		apply_material(sc->mMaterials[mesh->mMaterialIndex]);

		if(mesh->mNormals == NULL) {
			glDisable(GL_LIGHTING);
		} else {
			glEnable(GL_LIGHTING);
		}

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;

			switch(face->mNumIndices) {
				case 1: face_mode = GL_POINTS; break;
				case 2: face_mode = GL_LINES; break;
				case 3: face_mode = GL_TRIANGLES; break;
				default: face_mode = GL_POLYGON; break;
			}

			glBegin(face_mode);

			for(i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
				if(mesh->mColors[0] != NULL)
					glColor4fv((GLfloat*)&mesh->mColors[0][index]);
				if(mesh->mNormals != NULL)
					glNormal3fv(&mesh->mNormals[index].x);
				glVertex3fv(&mesh->mVertices[index].x);
			}

			glEnd();
		}

	}

	/* draw all children */
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursive_render(sc, nd->mChildren[n]);
	}

	glPopMatrix();
}

void draw_charts (const chart_list all_charts)
{
	aiMatrix4x4 m = scene->mRootNode->mTransformation;
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);
	glDisable(GL_LIGHTING);
	if(b_line_mode)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	int len = all_charts.size();
	for (int c=0; c<len; c++) {
		const aiMesh* mesh = all_charts[c].mi.mesh;
		//cout<<c<<"-"<<len;
		float r = float(c)/len;
		float g = 1-4*pow(r-0.5,2);
		float b = c%2==0? r : 1-r;
		glColor3f(r,g,b);
		
		for (vector<chart_face>::const_iterator it=all_charts[c].faces.begin(); it!=all_charts[c].faces.end(); it++) {
			glBegin(GL_TRIANGLES);
			for(int i = 0; i < 3; i++) {
				int id_on_mesh = all_charts[c].vertices[it->indices[i]];
				glVertex3fv(&mesh->mVertices[id_on_mesh].x);
			}
			glEnd();
		}
		glColor3f(1.0,1.0,1.0);
	}

	glEnable(GL_LIGHTING);
	glPopMatrix();
}

void draw_split_edges() {
	aiMatrix4x4 m = scene->mRootNode->mTransformation;
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);

	glDisable(GL_LIGHTING); //light and color can not be used at the same time
	glColor3f(1.0,0.0,0.0);
	glBegin(GL_LINES);  

	int index = 0;
	for(int me=0; me<scene->mNumMeshes; me++) {
		const aiMesh* mesh = scene->mMeshes[me];	
		edge_list es = all_split_edges[index];
		for(edge_list::iterator it = es.begin(); it != es.end(); it++) {
			edge e = *it;
			glVertex3fv(&mesh->mVertices[e.pA].x);
			glVertex3fv(&mesh->mVertices[e.pB].x);
			/*cout<<e.pA<<"---"<<e.pB<<endl;*/
		}
		index++;
	}

	glEnd();
	glColor3f(1.0,1.0,1.0);
	glEnable(GL_LIGHTING);

	glPopMatrix();
}

void draw_UVs() {
	glDisable(GL_LIGHTING); //light and color can not be used at the same time
	glColor3f(0.0,1.0,0.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glBegin(GL_POLYGON);
	glVertex3f(0.f,0.f,0.f);
	glVertex3f(1.f,0.f,0.f);
	glVertex3f(1.f,1.f,0.f);
	glVertex3f(0.f,1.f,0.f);
	glEnd();

	for(int c=0; c<packed_all_UV.size(); c++) {
		const UV_list UV = packed_all_UV[c];
		const chart *cp = &all_charts[c];
		const aiMesh* mesh = cp->mi.mesh;
		for(vector<chart_face>::const_iterator it=cp->faces.begin(); it!=cp->faces.end(); it++) {
			glBegin(GL_TRIANGLES);  
			for(unsigned int i = 0; i < 3; i++) {
				int new_id = it->indices[i];
				//cout<<" i "<<new_id<<" u "<<UV[new_id].x<<" v "<<UV[new_id].y;
				glVertex3f(UV[new_id].x(),UV[new_id].y(),0);
			}
			glEnd();
		}
	}

	glColor3f(1.0,1.0,1.0);
	glEnable(GL_LIGHTING);
}

/* ---------------------------------------------------------------------------- */
void do_motion (void)
{
	int time = glutGet(GLUT_ELAPSED_TIME);
	angle += (time-prev_time)*0.02;
	prev_time = time;

	glutPostRedisplay ();
}

/* ---------------------------------------------------------------------------- */
void grab(const char* filename)
{
    FreeImage_Initialise();
    
    unsigned char *mpixels =new unsigned char[WINDOW_WIDTH * WINDOW_HEIGHT * 4];
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, mpixels);
    glReadBuffer(GL_BACK);
    for(int i=0; i<(int)WINDOW_WIDTH*WINDOW_HEIGHT*4; i+=4)
    { 
        mpixels[i] ^= mpixels[i+2] ^= mpixels[i] ^= mpixels[i+2];
    }
    FIBITMAP* bitmap =FreeImage_Allocate(WINDOW_WIDTH, WINDOW_HEIGHT, 32, 8, 8, 8);

    for(int y = 0 ;y < FreeImage_GetHeight(bitmap); y++)
    {
        BYTE *bits = FreeImage_GetScanLine(bitmap, y);
        for(int x = 0; x < FreeImage_GetWidth(bitmap); x++)
        {
            bits[0] = mpixels[(y*WINDOW_WIDTH+x)*4+0];
            bits[1] = mpixels[(y*WINDOW_WIDTH+x)*4+1];
            bits[2] = mpixels[(y*WINDOW_WIDTH+x)*4+2];
            bits[3] = 255;
            bits += 4;
        }
    }
    bool bSuccess = FreeImage_Save(FIF_PNG, bitmap, filename, PNG_DEFAULT);
    FreeImage_Unload(bitmap);
    
    FreeImage_DeInitialise();
}

/* ---------------------------------------------------------------------------- */
void display(void)
{
	float tmp;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	if(b_UV_mode) {
		gluLookAt(UV_eye[0], UV_eye[1], UV_eye[2],
			UV_center[0], UV_center[1], UV_center[2],
			0.f, 1.f, 0.f);
	}
	else {
		gluLookAt(eye[0], eye[1], eye[2],
			center[0], center[1], center[2],
			0.f, 1.f, 0.f);
		glRotatef(angle,0.f,1.f,0.f); /* rotate it around the y axis */

		/* scale the whole asset to fit into our view frustum */
		tmp = scene_max.x-scene_min.x;
		tmp = aisgl_max(scene_max.y - scene_min.y,tmp);
		tmp = aisgl_max(scene_max.z - scene_min.z,tmp);
		tmp = 1.f / tmp;
		glScalef(tmp, tmp, tmp);

		/* center the model */
		glTranslatef( -scene_center.x, -scene_center.y, -scene_center.z );
	}
	
	if(b_UV_mode) {
		draw_UVs();
	}
	else {
			/* if the display list has not been made yet, create a new one and
			fill it with scene contents */
		/*if(scene_list == 0) {
			scene_list = glGenLists(1);
			glNewList(scene_list, GL_COMPILE);*/
				/* now begin at the root node of the imported data and traverse
				the scenegraph by multiplying subsequent local transforms
				together on GL's matrix stack. */
			if(b_chart_mode && all_charts.size()>0)
				draw_charts(all_charts);
			else
				recursive_render(scene, scene->mRootNode);
			/*glEndList();
		}
		glCallList(scene_list);*/

		if(all_split_edges.size()>0) {
			draw_split_edges();
		}
	}

	glutSwapBuffers();

    if (b_rotate) {
        do_motion();	    
    }
}

void key(unsigned char k, int x, int y)
{
	switch(k)
	{
	    case 27: { exit(0); break; } /* press esc to quit */
        case 'a': { if(!b_UV_mode) {eye[0]+=0.05; center[0]+=0.05;} else {UV_eye[0]+=0.02; UV_center[0]+=0.02;} break; }
	    case 'd': { if(!b_UV_mode) {eye[0]-=0.05; center[0]-=0.05;} else {UV_eye[0]-=0.02; UV_center[0]-=0.02;} break; }
    	case 'w': { if(!b_UV_mode) {eye[1]-=0.05; center[1]-=0.05;} else {UV_eye[1]-=0.02; UV_center[1]-=0.02;} break; }
	    case 's': { if(!b_UV_mode) {eye[1]+=0.05; center[1]+=0.05;} else {UV_eye[1]+=0.02; UV_center[1]+=0.02;} break; }
	    case 'z': { if(!b_UV_mode) {eye[2]-=0.05; center[2]-=0.05;} else {UV_eye[2]-=0.02; UV_center[2]-=0.02;} break; }
	    case 'c': { if(!b_UV_mode) {eye[2]+=0.05; center[2]+=0.05;} else {UV_eye[2]+=0.02; UV_center[2]+=0.02;} break; }
        case ' ': { if(!b_UV_mode) {b_rotate = !b_rotate; prev_time = glutGet(GLUT_ELAPSED_TIME);} break; }
		case 'l': { if(!b_UV_mode) {b_line_mode = !b_line_mode;} break; }
		case 'k': { if(!b_UV_mode) {b_chart_mode = !b_chart_mode;} break; }
	    case 'e': { 
			scene_segment( scene_info(scene,all_edges,all_efm,all_refm,all_vem,all_fn) ,all_split_edges, all_charts ); 
			break; 
		}
		case 'p': { 
			scene_parameterize( scene_info(scene,all_edges,all_efm,all_refm,all_vem,all_fn), all_charts, all_UV, PARAM_OPENNL );
			rectangle_pack(all_charts,all_UV,packed_all_UV);
			break; 
		}
		case 'q': {
			scene_parameterize( scene_info(scene,all_edges,all_efm,all_refm,all_vem,all_fn), all_charts, all_UV, PARAM_EIGEN );
			rectangle_pack(all_charts,all_UV,packed_all_UV);
			break; 
		}
		case 'u': { b_UV_mode = !b_UV_mode; break; }
	    case 'g': { grab("test.png"); break; }
		case 'o': { gen_obj( all_charts, packed_all_UV ); break; }
    }
}

void idle()
{
	glutPostRedisplay();
}

/* ---------------------------------------------------------------------------- */
void pretreat()
{
	int m_num=0, f_num=0, e_num=0, v_num=0;	
	for(int m=0; m<scene->mNumMeshes; m++) {
		cout<<"mesh "<<m<<" :"<<endl;
		const aiMesh* mesh = scene->mMeshes[m];
		edge_list el;
		edge_face_map efm;
		raw_edge_face_map refm;
		vertex_edge_map vem;
		face_normals fn;
		gen_edges(mesh,el);
		cout<<"gen_edges() done"<<endl;
		gen_edge_face_map(mesh,el,efm,refm);
		cout<<"gen_edge_face_map() done"<<endl;
		gen_vertex_edge_map(el,vem);
		cout<<"gen_vertex_edge_map() done"<<endl;
		gen_face_normals(mesh,fn);
		cout<<"gen_face_normals() done"<<endl;
		all_edges.push_back(el);
		all_efm.push_back(efm);
		all_refm.push_back(refm);
		all_vem.push_back(vem);
		all_fn.push_back(fn);
		m_num += 1;
		f_num += mesh->mNumFaces;
		e_num += el.size();
		v_num += mesh->mNumVertices;
	}
	printf("Found %d meshes, %d faces, %d edges, and %d vertices.\n",m_num,f_num,e_num,v_num);
}

int loadasset (const char* path)
{
	/* we are taking one of the postprocessing presets to avoid
	   spelling out 20+ single postprocessing flags here. */
	scene = aiImportFile(path,aiProcessPreset_TargetRealtime_MaxQuality);

	if (scene) {
		get_bounding_box(&scene_min,&scene_max);
		scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
		scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
		scene_center.z = (scene_min.z + scene_max.z) / 2.0f;

		if(scene->mRootNode->mNumChildren>0) {
			printf("WARNING: Do not support hierarchy of nodes!\n");
		}
		pretreat();

		return 0;
	}
	return 1;
}

/* ---------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
	aiLogStream stream;

	nlInitialize(argc, argv);

	glutInitWindowSize(WINDOW_WIDTH,WINDOW_HEIGHT);
	glutInitWindowPosition(100,100);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInit(&argc, argv);
	glutCreateWindow("myUVlayout");

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
    glutKeyboardFunc(key);
    glutIdleFunc(idle);

	/* get a handle to the predefined STDOUT log stream and attach
	   it to the logging system. It remains active for all further
	   calls to aiImportFile(Ex) and aiApplyPostProcessing. */
	stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
	aiAttachLogStream(&stream);

	/* ... same procedure, but this stream now writes the
	   log messages to assimp_log.txt */
	stream = aiGetPredefinedLogStream(aiDefaultLogStream_FILE,"myUVlayout_log.txt");
	aiAttachLogStream(&stream);

	/* the model name can be specified on the command line */
	if( 0 != loadasset(argv[1]) ) {    /* loadasset() is the key */
		return -1;
	}

	glClearColor(0.1f,0.1f,0.1f,1.f);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);    /* Uses default lighting parameters */
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_NORMALIZE);

	/* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
	if(getenv("MODEL_IS_BROKEN"))
		glFrontFace(GL_CW);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	glutGet(GLUT_ELAPSED_TIME);
	glutMainLoop();

	/* cleanup - calling 'aiReleaseImport' is important, as the library
	   keeps internal resources until the scene is freed again. Not
	   doing so can cause severe resource leaking. */
	aiReleaseImport(scene);

	/* We added a log stream to the library, it's our job to disable it
	   again. This will definitely release the last resources allocated
	   by Assimp.*/
	aiDetachAllLogStreams();
	return 0;
}
