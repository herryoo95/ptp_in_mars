#include "PTPopenGL.hpp"


namespace soildynamics {
//}// end of namespace soildynamics


PTPSystem* ptpSys =new PTPSystem();
KalmanFilter* kf=new KalmanFilter();

GLfloat left, right, bottom, top, Near, Far;
GLfloat fov;
GLsizei lastWidth, lastHeight;
GLfloat xAngle, yAngle, zAngle, magScale, xTrans, yTrans, zTrans;
GLint xx, yy, lClick, rClick;

int Projection;
int Object;
	int _InputState;
int t_J, t_K, t_debug;

#ifdef SPHERE_EXPERIMENT2
int t_p, t_c, t_f, t_b, t_a, t_x, t_d, t_t, t_s, t_j, t_k, t_m, t_v;
#endif
#ifndef SPHERE_EXPERIMENT2
int t_p, t_c, t_f, t_b, t_a, t_x, t_d, t_t, t_s, t_j, t_k, t_m, t_v;
#endif

int v_x,v_y,v_z;
FOOT foot;
FOOT footLast;
vecD forceR;
vecD forceT;

int step;
double step_size;
double msec;
double vel_set;
FILE *out;
FILE *out_cell;


float q,w,e;
EXPDATA edata[2500];     /** loading lookup all file data. EXPDATA is defined in PTPopenGL.hpp*/ 


char filename[150];
char filename_cell[153];


void PTPopenGL()
{
	step=0;
	left=-1;
	right=1;
	bottom=-1;
	top=1;
	Near=-1.0;
	Far=1.0;
	fov = 60;

	xAngle=0;
	yAngle=0;
	zAngle=0;
	magScale=0.02;
	xTrans=0;
	yTrans=0;
	zTrans=0;
	xx=0;
	yy=0;
	lClick=0;
	rClick=0;
	_InputState = I_NON;
	t_J=0;
	t_K=0;
	t_debug=0;
	#ifdef SPHERE_EXPERIMENT2
		t_p=0;
		t_c=0;
		t_f=1;
		t_b=1;
		t_a=0; 
		t_x=1;
		t_d=1;
		t_t=0;
		t_s=0;
		t_j=0;
		t_k=0;
		t_m=5;
		t_v=0;
	#endif
	#ifndef SPHERE_EXPERIMENT2
		t_p=0;
		t_c=0;
		t_f=2;
		t_b=1;
		t_a=0;
		t_x=1;
		t_d=1;
		t_t=0;
		t_s=0;
		t_j=0;
		t_k=0;
		t_m=5;
		t_v=0;
	#endif
	v_x=0;
	v_y=0;
	v_z=0;

	step=0;
	step_size=0.01f;  //sec
	msec=0.f;
	vel_set = 1.f;

}



void DoMenu(int value)
{
	switch(value) {
	case 1:
		if(t_f==3){	t_f=0; }else{t_f++;}break;
	case 2:
		if(t_p==3){	t_p=0;}else{t_p++;}break;
	case 3:
		if(t_c==5){	t_c=0;}else{t_c++; printf("add\n");}break;
	case 4:
		if(t_x==1){	t_x=0;}else{t_x++;}	break;
	case 5:
		if(t_d==1){t_d=0;}else{	t_d++;}	break;
	case 6:
		if(t_m==6){t_m=0;}else{	t_m++;}	break;
	case 7:
		if(t_s==1){t_s=0;}else{	t_s++;}	break;
	case 8:
		if(t_v==1){t_v=0;}else{	t_v++;}	break;
	case 20:
		if(t_debug==6){t_debug=0;}else{t_debug++;}break;
	case 10:
		Projection = 0;
		Near = -1;
		Far = 1;
		break;
	case 11:
		Projection = 1;
		Near = 1;
		Far = 10;
		break;
	case 12:
		Projection = 2;
		Near = 1;
		Far = 10;
		break;
	case 13:
		if(t_b==1){	t_b=0;}else{t_b=1;}break;
	case 14:
		if(t_a==1){	t_a=0;}else{t_a=1;}break;
	}
	glutPostRedisplay();
}

void DoReshape(GLsizei width, GLsizei height)
{
	GLfloat aspect;

	glViewport(0,0,width,height);

	if (height == 0) return;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	aspect = (GLfloat)width / (GLfloat)height;
	if (width > height) {
		glOrtho(-1.0f*aspect, 1.0f*aspect, -1.0f, 1.0f, 1.0f, -1.0f);      
	} else {
		glOrtho(-1.0f, 1.0f, -1.0f/aspect, 1.0f/aspect, 1.0f, -1.0f);
	}

	glMatrixMode(GL_MODELVIEW);

	lastWidth = width;
	lastHeight = height;


}
void ReadExperimentData(int type){
	FILE *fp;
	double data[26];
	char buf[BUF_SIZE];

	char	*ptr;
	char *err;
	double f;
	int i,j=0;
	vecD filtered=ptpSys->ptputil->setVecD(0.f,0.f,0.f);

	filtered = kf->kalman(filtered,0,ptpSys->ptputil);
	if(type==0){
		fp=fopen("2011_05_04_14_12_14log.csv", "r");  // vertical exp.
	}
	if(type==1){
		fp=fopen("2011_04_05_11_50_22log.csv", "r");  // Lateral exp.
	}
	if(type==0||type==1){
		while(!feof(fp))  
		{
			//printf("\n",f);
			fgets(buf, BUF_SIZE, fp);  
			if(buf[0]=='%' || buf[0] == 10) continue;
			i=0;
			ptr = strtok( buf, "; ");
			do{

				f = strtod( ptr, &err);
				data[i] = f;
		//		printf("%f %s\t",f,err);
				i++;
			}while(ptr = strtok(NULL, "; "));
			edata[j].msec = (int)data[0];
			edata[j].Joint0 = data[1];
			edata[j].Joint1 = data[2];
			edata[j].Joint2 = data[3];
			edata[j].Joint3 = data[4];
			edata[j].Joint4 = data[5];
			edata[j].Joint5 = data[6];
			edata[j].Joint6 = data[7];
			edata[j].Fx = data[8];
			edata[j].Fy = data[9];
			edata[j].Fz = data[10];
			edata[j].Tx = data[11];
			edata[j].Ty = data[12];
			edata[j].Tz = data[13];
			edata[j].FTStatus = (int)data[14];
			edata[j].RTDSeq = (int)data[15];
			edata[j].FTSeq = (int)data[16];
			edata[j].Marker = (int)data[17];
			edata[j].currentPointID = (int)data[18];
			//filtered = kf->kalman(setVecD(data[19],data[20],data[21]),1);
			edata[j].X = data[19];
			edata[j].Y = data[20];
			edata[j].Z = data[21];
			edata[j].Rx = data[22];
			edata[j].Ry = data[23];
			edata[j].Rz = data[24];
		
	//	printf("..%f\n", edata[j].Fy);
			j++;
		}

		fclose(fp);
	}
}
void DoKeyboard(unsigned char key, int x, int y)
{
	int flag=0;
	switch(key) {

	case 'r':
	case 'R': //ROTATION
		_InputState = I_ROT;
		break;
	case 't':
	case 'T': //TRANSLATION
		_InputState = I_TRS;
		break;
	case 'z':
	case 'Z': //MAGNIFI
		_InputState = I_MAG;
		break;
	case 'f':
		_InputState = I_NON;
		xAngle = yAngle = zAngle = xTrans = yTrans = zTrans = 0.f;
		magScale = 0.1f;
		break;
	case 27:
		_InputState = I_NON;

		break;
	case '-':
		if(step_size > 0.01f){
			step_size -= 0.01f;
		}
		break;
	case '+':
		step_size += 0.01f;
		break;
	case ',':
		if(v_x==ptpSys->ptp->n.x){v_x=0;}else{v_x++;}break;

	case '.':
		if(v_y==ptpSys->ptp->n.y){v_y=0;}else{v_y++;}break;
	case '/':
		if(v_z==ptpSys->ptp->n.z){v_z=0;}else{v_z++;}break;



		///For Debug
	case 'p':q+=0.1f;break;
	case '[':w+=0.1f;break;
	case ']':e+=0.1f;break;
	case 'l':q-=0.1f;break;
	case ';':w-=0.1f;break;
	case '\'':e-=0.1f;break;


	case 'h':		/// CYLINDER 
		if(t_t==0){t_t=1;}else{t_t=0;}break;

	case 'j':		/// SPHARE
		if(t_j==0){t_j=1;}else{t_j=0;}break;
	case 'J':
		ReadExperimentData(0);
		if(t_J==0){t_J=1;}else{t_J=0;}break;
	case 'k':		/// SPHARE
		if(t_k==0){t_k=1;}else{t_k=0;}break;

	case 'K':		/// SPHARE
		ReadExperimentData(1);
		if(t_K==0){t_K=1;}else{t_K=0;}break;
	default:
		flag=0;
	}


	switch(key){	
	case 'a':
		foot.vel.x = -vel_set;
		foot.vel.y = 0;
		foot.vel.z = 0;
		flag=1;
		break;
	case 'A':
		foot.vel.x -= vel_set;
		flag=1;
		break;
	case 'd':
		foot.vel.x = vel_set;
		foot.vel.y = 0;
		foot.vel.z = 0;
		flag=1;
		break;
	case 'D':
		foot.vel.x += vel_set;
		flag=1;
		break;
	case 'w':
		foot.vel.x = 0;
		foot.vel.y = vel_set;
		foot.vel.z = 0;
		flag=1;
		break;
	case 'W':
		foot.vel.y += vel_set;
		flag=1;
		break;
	case 's':
		foot.vel.x = 0;
		foot.vel.y = -vel_set;
		foot.vel.z = 0;
		flag=1;
		break;
	case 'S':
		foot.vel.y -= vel_set;
		flag=1;
		break;
	case 'e':
		foot.vel.x = 0;
		foot.vel.y = 0;
		foot.vel.z = vel_set;
		flag=1;
		break;
	case 'E':
		foot.vel.z += vel_set;
		flag=1;
		break;
	case 'q':
		foot.vel.x = 0;
		foot.vel.y = 0;
		foot.vel.z = -vel_set;
		flag=1;
		break;
	case 'Q':
		foot.vel.z -= vel_set;
		flag=1;
		break;
	case 'x':
	case 'X':
		foot.vel.x = 0;
		foot.vel.y = 0;
		foot.vel.z = 0;
		break;
	}

	if(flag==1){
		foot.loc.x += step_size*foot.vel.x;    // no velocity effect 
		foot.loc.y += step_size*foot.vel.y;		//10 mm/step (if vel = 1 m/s, setp_size = 0.01 s) 
		foot.loc.z += step_size*foot.vel.z;

		if(!ptpSys->getPTPForce(foot,&forceR,&forceT)) {printf("ERROR!!!");}else{
			out = fopen(filename, "a+");
			msec += step_size*100;
			fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",msec*10,foot.loc.y,forceR.y,forceR.z,forceT.x,forceT.y,forceT.z,foot.loc.x,foot.loc.y,foot.loc.z);
			fclose(out); 
			step++;
		}
	}

	footLast = foot;
	glutPostRedisplay();
}


void MyTimer(int Value)
{
	static int status=0;
	if(Value==0){	//CYLINDER 
		if(foot.loc.y<=-30.f) {t_t=0;}
		foot.vel.x = 0.f;
		foot.vel.y = -5.f;
		foot.vel.z = 0.f;
	}
	if(Value==1){	//SPHARE 
		if(foot.type==SPHERE){
			switch(status){
			case 0:
				foot.vel.x = 0.f;
				foot.vel.y = 0.f;  //yz
				foot.vel.z = -1.f;
				if(foot.loc.z-foot.r<=-10.f) status =1;   //yz

				break;
			case 1:
				foot.vel.x = 0.f;
				foot.vel.y = 0.f;
				foot.vel.z = 1.f;
				if(foot.loc.z-foot.r>=-5.f) status =2;   //yz

				break;
			case 2:
				foot.vel.x = 0.f;
				foot.vel.y = 0.f;
				foot.vel.z = -1.f;
				if(foot.loc.z-foot.r<=-30.f) status =3;   //yz
				break;
			case 3:
				foot.vel.x = 0.f;
				foot.vel.y = 0.f;
				foot.vel.z = 0.f;
				t_j=0;
				break;
			}}else{
				switch(status){
				case 0:
					foot.vel.x = 0.f;
					foot.vel.y = 0.f;
					foot.vel.z = -1.f;
					if(foot.loc.z-(foot.l/2)<=-10.f) status =1;    //yz
					break;
				case 1:
					foot.vel.x = 0.f;
					foot.vel.y = 0.f;
					foot.vel.z = 1.f;
					if(foot.loc.z-(foot.l/2)>=-5.f) status =2;  	//yz
					break;
				case 2:
					foot.vel.x = 0.f;
					foot.vel.y = 0.f;
					foot.vel.z = -1.f;

					if(foot.loc.z-(foot.l/2)<=-30.f) status =3;     //yz
					break;
				case 3:
					foot.vel.x = 0.f;
					foot.vel.y = 0.f;
					foot.vel.z = 0.f;
					t_j=0;
					break;


				}
			}


	}
	if(Value==2){		
		switch(status){
		case 0:
			foot.vel.x = 0.f;
			foot.vel.y = 0.f;    
			foot.vel.z = -5.f;
			if(foot.loc.z-foot.r<=-20.f) status =1;   	//yz
			break;
		case 1:
			foot.vel.x = 12.5f;
			foot.vel.y = 0.f;
			foot.vel.z = 0.25f;  		//yz
			if(foot.loc.x-foot.r>=150.f) status =2;
			break;
		case 2:
			foot.vel.x = 0.f;
			foot.vel.y = 0.f;
			foot.vel.z = 0.f;
			t_k=0;
			break;
		}
	}

	foot.loc.x += step_size*foot.vel.x;								///[mm/s]
	foot.loc.y += step_size*foot.vel.y;
	foot.loc.z += step_size*foot.vel.z;


	if(!ptpSys->getPTPForce(foot,&forceR,&forceT)){printf("ERRRROR");}else{
		out = fopen(filename, "a+");
		msec += step_size*100;

		fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",msec*10,forceR.x,forceR.y,forceR.z,forceT.x,forceT.y,forceT.z,foot.loc.x,foot.loc.y,foot.loc.z);
	
		fclose(out); 
		step++;
	}
	footLast = foot;
	glutPostRedisplay();
	glutTimerFunc(1, MyTimer, Value);	
}



void MyTimer2(int Value)
{
	static int status=0;
	static int i=0;
	vecD filter=ptpSys->ptputil->setVecD(0.f,0.f,0.f);
	double buf;

	if(status==0){
		kf->kalman(filter,0,ptpSys->ptputil);
		if(Value==0){
			i=860;
		}
		if(Value==1){
			i=678;//580;
		}
	}
	if(Value==0){	// sphere vertical
		if(i==1926) {i=0; t_J=0;status=0;}else{    // t_J == 0: using vertical exp.	
			foot.loc.x = edata[i].X+239.1;
			foot.loc.z = edata[i].Y+331.1+foot.r;    //yz
			foot.loc.y = edata[i].Z-583.2;			 //zy

			filter.x = (foot.loc.x - footLast.loc.x)*1000;///(edata[i].msec-edata[i-1].msec);
			filter.y = (foot.loc.y - footLast.loc.y)*1000;//(edata[i].msec-edata[i-1].msec);
			filter.z = (foot.loc.z - footLast.loc.z)*1000;//(edata[i].msec-edata[i-1].msec);
			filter = kf->kalman(filter,1,ptpSys->ptputil);
			buf = kf->kalmanD((edata[i].msec-edata[i-1].msec),1);
			foot.vel.x = filter.x/(buf);
			foot.vel.y = filter.y/(buf);
			foot.vel.z = filter.z/(buf);

			status =1;
			i++;
		}
	} 
	if(Value==1){	// sphere lateral
		if(i==1300) {i=0; t_K=0;status=0;}else{
			foot.loc.x = edata[i].X+230;
			foot.loc.z = edata[i].Y+335+foot.r;     //yz   
			foot.loc.y = edata[i].Z-589;			//zy
			filter.x = (foot.loc.x - footLast.loc.x)*1000;///(edata[i].msec-edata[i-1].msec);
			filter.y = (foot.loc.y - footLast.loc.y)*1000;//(edata[i].msec-edata[i-1].msec);
			filter.z = (foot.loc.z - footLast.loc.z)*1000;//(edata[i].msec-edata[i-1].msec);
			filter = kf->kalman(filter,1,ptpSys->ptputil);

			buf = kf->kalmanD((edata[i].msec-edata[i-1].msec),1);
			foot.vel.x = filter.x/(buf);
			foot.vel.y = filter.y/(buf);
			foot.vel.z = filter.z/(buf);

			status =1;
			i++;
		}
	}



	if(!ptpSys->getPTPForce(foot,&forceR,&forceT)){printf("ERRRROR");}else{

		if(status==1){
			out=fopen(filename, "a+");
			if(Value==0)
				msec = (edata[i].msec-35200)/10;
			if(Value==1)
				msec = (edata[i].msec-26900)/10;

			fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, %f, %f, %f\n",msec*10,forceR.x,forceR.y,forceR.z,forceT.x,forceT.y,forceT.z,foot.loc.x,foot.loc.y,foot.loc.z,foot.vel.x,foot.vel.y,foot.vel.z);
			fclose(out); 
			step++;
		}
		footLast = foot;
		glutPostRedisplay();
		glutTimerFunc(1, MyTimer2, Value);	
	}
}


void DoSpecial(int key, int x, int y)
{
	switch(key) {
	case GLUT_KEY_LEFT:
		if(_InputState==I_MAG) magScale -= 0.1;
		if(_InputState==I_TRS) xTrans -= 0.1;
		if(_InputState==I_ROT) yAngle -= 1;
		break;
	case GLUT_KEY_RIGHT:
		if(_InputState==I_MAG) magScale += 0.1;
		if(_InputState==I_TRS) xTrans += 0.1;
		if(_InputState==I_ROT) yAngle += 1;
		break;
	case GLUT_KEY_UP:
		if(_InputState==I_MAG) magScale *= 1.1;
		if(_InputState==I_TRS) yTrans += 0.1;
		if(_InputState==I_ROT) xAngle += 1;

		break;
	case GLUT_KEY_DOWN:
		if(_InputState==I_MAG) magScale *= 0.9;
		if(_InputState==I_TRS) yTrans -= 0.1;
		if(_InputState==I_ROT) xAngle -= 1;
		break;
	}
	glutPostRedisplay();
}

void DoMouse(int button, int state, int x, int y)
{
	y = lastHeight - y;

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		lClick = 1;
	}	else if(button==GLUT_RIGHT_BUTTON && state == GLUT_DOWN){
		rClick = 1;
	} else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
		lClick = 0;
	}else if(button == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
		rClick = 0;
	}else
	{
		lClick = rClick = 0;
	}	
	xx = x;
	yy = y;
}

void DoMouseMotion(int x, int y)
{
	y = lastHeight - y;
	if(lClick)
	{

		if(_InputState==I_MAG && (x-xx)+(y-yy)>0) magScale *= 1.1;
		if(_InputState==I_MAG && (x-xx)+(y-yy)<0) magScale *= 0.9;

		if(_InputState==I_TRS) {
			if((x-xx)>0)xTrans += 0.02;
			if((y-yy)>0)yTrans += 0.02;
			if((x-xx)<0)xTrans -= 0.02;
			if((y-yy)<0)yTrans -= 0.02;
		}
		if(_InputState==I_ROT) {yAngle -= (x - xx)/3.6; xAngle += (y - yy)/3.6;}
	}

	else if(rClick)
	{

	}
	xx = x;
	yy = y;
	glutPostRedisplay();
}

static void out_string_at(void *font, float x, float y, float z, char *string)
{
#ifdef __FREEGLUT_EXT_H__
	// this is ONLY if GLUT has this extended mode
	glRasterPos3f((GLfloat)x, (GLfloat)y, (GLfloat)z);
	glutBitmapString( font, (const unsigned char *)string );
#else
	int len, i;
	glRasterPos3f((GLfloat)x, (GLfloat)y, (GLfloat)z); // set position
	len = (int) strlen(string);
	for (i = 0; i < len; i++) { // output character by character
		glutBitmapCharacter(font, string[i]);
	}
#endif
}


void DoDisplay(void)
{
	char msg[500];
	int i,j,k;
	vecD re;
	vecD r;
	GLfloat aspect;
	GLfloat light0_pos[] = {1.0, 2.0, 3.0, 0.0};
	GLfloat diffuse0[] = {0.9, 1.0 ,0.7, 0.5};
	GLfloat ambient0[] = {0.7, 0.7 ,0.4, 0.0};
	GLfloat specular0[] = {1.0, 1.0, 1.0, 1,0};
	GLenum Src = GL_SRC_ALPHA;
	GLenum Dest = GL_ONE_MINUS_SRC_ALPHA;

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);  
	glEnable(GL_LIGHTING);  
	glEnable(GL_LIGHT0);  
	glLightfv(GL_LIGHT0, GL_POSITION, light0_pos); // 
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient0); // 
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0); // 
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular0); // 

	//	glEnable(GL_NORMALIZE);  

	if(t_t==1){
		glutTimerFunc(1, MyTimer, 0); 
	}

	if(t_j==1){
		glutTimerFunc(1, MyTimer, 1); 
	}
	if(t_k==1){
		glutTimerFunc(1, MyTimer, 2); 
	}
	if(t_J==1){
		glutTimerFunc(1, MyTimer2, 0);
	}
	if(t_K==1){
		glutTimerFunc(1, MyTimer2, 1);
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	if(t_b){
		glEnable(GL_BLEND);
		Src = GL_SRC_ALPHA;
		Dest = GL_ONE_MINUS_SRC_ALPHA;
		glBlendFunc(Src, Dest);
		glShadeModel(GL_SMOOTH);


	}else{
		Src = GL_ONE;
		Dest = GL_ZERO;
	}

	// Anti-Alising on, off
	if (t_a) {
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_POLYGON_SMOOTH);
	} else {
		glDisable(GL_POINT_SMOOTH);
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_POLYGON_SMOOTH);
	}




	// Projection Mode 
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();			 
	aspect = (GLfloat)lastWidth / (GLfloat)lastHeight;
	switch (Projection) {
	case 0:
		if (lastWidth > lastHeight) {
			glOrtho(-1.0*aspect, 1.0*aspect, -1.0, 1.0, magScale*5000, -magScale*5000);
		} else {
			glOrtho(-1.0, 1.0, -1.0/aspect, 1.0/aspect, magScale*5000, -magScale*5000);
		}
		break;
	case 1:

		if (lastWidth > lastHeight) {
			//		glOrtho(-1.0*aspect, 1.0*aspect, -1.0, 1.0, magScale*5000, -magScale*5000);
			glFrustum(-1.0*aspect, 1.0*aspect, -1.0, 1.0, Near, Far);

		} else {
			//		glOrtho(-1.0, 1.0, -1.0/aspect, 1.0/aspect, magScale*5000, -magScale*5000);
			glFrustum(-1.0, 1.0, -1.0/aspect, 1.0/aspect,  Near, Far);

		}
		break;
	case 2:
		gluPerspective(fov, aspect, Near, Far);
		break;
	}


	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	if (Projection != 0) {
		glTranslatef(0,0,-1.5);
	}
	//fopen_s(&out,filename, "a+");

	/// T-R-S
	// Translate
	glTranslatef(xTrans, yTrans, zTrans);
	// Rotate
	glRotatef(xAngle, 1.0f, 0.0f, 0.0f);
	glRotatef(yAngle, 0.0f, 1.0f, 0.0f);
	glRotatef(zAngle, 0.0f, 0.0f, 1.0f);
	// Scaling
	glScalef(magScale, magScale, magScale);


	//foot
	if(t_f){
		glPushMatrix();

		glTranslatef(foot.loc.x, foot.loc.y, foot.loc.z);
		glColor3f( 0.01, 0, 0.01);     
		if(foot.type==SPHERE){
			if(t_f==3)glutSolidSphere(foot.r,50,50);
			if(t_f==2)glutWireSphere(foot.r,50,50);
			if(t_f==1)glutWireSphere(foot.r,20,20);
		}else if(foot.type == CYLINDER){
			GLUquadricObj *pQuad;
			int type;
			if(t_f==3) type=GLU_FILL;
			if(t_f==2) type=GLU_LINE;
			if(t_f==1) type=GLU_SILHOUETTE;
			pQuad = gluNewQuadric();
			gluQuadricDrawStyle(pQuad, type);//GLU_FILL GLU_LINE GLU_SILHOUETTE GLU_POINT
			glTranslatef(0,-foot.l/2,0);
			glRotatef(-90,1,0,0);
			gluCylinder(pQuad, foot.r, foot.r, foot.l, 20, 20);
		}else if(foot.type == CUBE){
			glScalef(foot.r,foot.l,foot.d);
			if(t_f==2)		glutWireCube(1);
			if(t_f==1)		glutSolidCube(1);
		}
		glPopMatrix();
	}


	glPushMatrix();
	for(i = 0;i<ptpSys->ptp->n.x;i++){
		for(j = 0;j<ptpSys->ptp->n.y;j++){
			for(k = 0;k<ptpSys->ptp->n.z;k++){
				glPushMatrix();
				if(t_m){
					//Movement
					re = ptpSys->particleMovement(&ptpSys->particle[i][j][k].loc,&foot);

					if(t_m==2){
						glBegin( GL_LINES );
						glColor3f(0,1,0); 
						glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG); 
						glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG)+re.x, (ptpSys->particle[i][j][k].loc.y/MAG)+re.y, (ptpSys->particle[i][j][k].loc.z/MAG)+re.z);
						glEnd();
						glPointSize(1.0f);     
						glBegin(GL_POINTS); 
						glColor3f( 1, 0, 0 );      
						glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG), (ptpSys->particle[i][j][k].loc.y/MAG), (ptpSys->particle[i][j][k].loc.z/MAG));
						glEnd();
					}else if(t_m==1){
						//if(ptpSys->particle[i][j][k].isCollision){
						glBegin( GL_LINES );
						glColor3f(0,1,0); 
						glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);
						glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG)+re.x, (ptpSys->particle[i][j][k].loc.y/MAG)+re.y, (ptpSys->particle[i][j][k].loc.z/MAG)+re.z);
						glEnd();
						glPointSize(1.0f);     
						glBegin(GL_POINTS); 
						glColor3f( 1, 0, 0 );      
						glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG), (ptpSys->particle[i][j][k].loc.y/MAG), (ptpSys->particle[i][j][k].loc.z/MAG));
						glEnd();
						//	}
					}else if(t_m==3){
						re = ptpSys->particleMovement(&ptpSys->cell[i][j][k].start,&foot);

						glBegin( GL_LINES );
						glColor3f(0,1,0); 
						glVertex3f(ptpSys->cell[i][j][k].start.x/MAG, ptpSys->cell[i][j][k].start.y/MAG, ptpSys->cell[i][j][k].start.z/MAG);  glVertex3f((ptpSys->cell[i][j][k].start.x/MAG)+re.x, (ptpSys->cell[i][j][k].start.y/MAG)+re.y, (ptpSys->cell[i][j][k].start.z/MAG)+re.z);
						glEnd();		
					}else if(t_m==4){
						if(i==v_x){
							glBegin( GL_LINES );
							glColor3f(0,1,0); 
							glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);  glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG)+re.x, (ptpSys->particle[i][j][k].loc.y/MAG)+re.y, (ptpSys->particle[i][j][k].loc.z/MAG)+re.z);
							glEnd();	
						}				
					}
					else if(t_m==5){
						if(j==v_y){
							glBegin( GL_LINES );
							glColor3f(0,1,0); 
							glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);  glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG)+re.x, (ptpSys->particle[i][j][k].loc.y/MAG)+re.y, (ptpSys->particle[i][j][k].loc.z/MAG)+re.z);
							glEnd();	
						}				
					}
					else if(t_m==6){
						if(k==v_z){
							glBegin( GL_LINES );
							glColor3f(0,1,0); 
							glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);  glVertex3f((ptpSys->particle[i][j][k].loc.x/MAG)+re.x, (ptpSys->particle[i][j][k].loc.y/MAG)+re.y, (ptpSys->particle[i][j][k].loc.z/MAG)+re.z);
							glEnd();	
						}				
					}

				}
				if(t_p){
					glPointSize(3.0f); 

					//PARTICLE
					if(t_p==2){ //ALL Particle
						glBegin(GL_POINTS); 
						if(i==0 && j==0 && k==0){
							glColor3f( 0, 1, 0 );      
						}else if(i==ptpSys->ptp->n.x-1 && j==ptpSys->ptp->n.y-1 && k==ptpSys->ptp->n.z-1){
							glColor3f( 0, 0, 1 );      
						}else{
							glColor3f( 0, 1, 0 );      
						}
						glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);   //��ǥ �ְ�

						glEnd();
					}else if(t_p==1){ //Only Collision Particle
						if(ptpSys->particle[i][j][k].isCollision){
							glBegin(GL_POINTS); 
							glColor3f( 1+(ptpSys->particle[i][j][k].loc.y/((ptpSys->ptp->end.y-ptpSys->ptp->start.y)*0.3)), -(ptpSys->particle[i][j][k].loc.y/(ptpSys->ptp->end.y-ptpSys->ptp->start.y)), 0 );     
							glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);   //��ǥ �ְ�
							glEnd();
						}
					}else if(t_p==3){ //Height Color
						glBegin(GL_POINTS); 
						glColor3f( 1+(ptpSys->particle[i][j][k].loc.y/((ptpSys->ptp->end.y-ptpSys->ptp->start.y)*0.3)), -(ptpSys->particle[i][j][k].loc.y/(ptpSys->ptp->end.y-ptpSys->ptp->start.y)), 0 );     
						glVertex3f(ptpSys->particle[i][j][k].loc.x/MAG, ptpSys->particle[i][j][k].loc.y/MAG, ptpSys->particle[i][j][k].loc.z/MAG);   //��ǥ �ְ�
						glEnd();
					}

				}
				if(ptpSys->cell[i][j][k].CollisionParticleNum){
					if(t_debug){
#ifdef USE_FORCE_DEBUG

						if(t_debug==5){


							glBegin( GL_LINES );
							glColor3f(1,0,0); 
							glVertex3f(ptpSys->cell[i][j][k].center.x/MAG,ptpSys->cell[i][j][k].center.y/MAG,	ptpSys->cell[i][j][k].center.z/MAG);
							glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fZ.x+ptpSys->cell[i][j][k].fL.x))/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fZ.y+ptpSys->cell[i][j][k].fL.y))/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fZ.z+ptpSys->cell[i][j][k].fL.z))/MAG);
							glEnd();

							//if(ptpSys->cell[i][j][k].fZ.x!=0 || ptpSys->cell[i][j][k].fZ.y !=0 || ptpSys->cell[i][j][k].fZ.z!=0 || ptpSys->cell[i][j][k].fL.x!=0 || ptpSys->cell[i][j][k].fL.y !=0 || ptpSys->cell[i][j][k].fL.z!=0){
							glBegin(GL_POINTS); 
							glPointSize(3.0f); 
							glColor3f(1,1,1);
							glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fZ.x+ptpSys->cell[i][j][k].fL.x))/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fZ.y+ptpSys->cell[i][j][k].fL.y))/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fZ.z+ptpSys->cell[i][j][k].fL.z))/MAG);

							glEnd();

							//						}
						}

						else if(t_debug==1){


							glBegin( GL_LINES );
							glColor3f(0,1,0); 
							glVertex3f(ptpSys->cell[i][j][k].center.x/MAG,ptpSys->cell[i][j][k].center.y/MAG,	ptpSys->cell[i][j][k].center.z/MAG);
            				glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fZ.x))/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fZ.y))/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fZ.z))/MAG);
							glEnd();

							//							if(ptpSys->cell[i][j][k].fZ.x!=0 || ptpSys->cell[i][j][k].fZ.y !=0 || ptpSys->cell[i][j][k].fZ.z!=0){
							glBegin(GL_POINTS); 
							glPointSize(3.0f); 
							glColor3f(1,0,0);
							glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fZ.x))/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fZ.y))/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fZ.z))/MAG);
							glEnd();
							//							}

						}
						else if(t_debug==2){


							glBegin( GL_LINES );
							glColor3f(0,0,1); 
							glVertex3f(ptpSys->cell[i][j][k].center.x/MAG,ptpSys->cell[i][j][k].center.y/MAG,	ptpSys->cell[i][j][k].center.z/MAG);
							glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fL.x))/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fL.y))/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fL.z))/MAG);
							glEnd();

							//if(ptpSys->cell[i][j][k].fL.x!=0 || ptpSys->cell[i][j][k].fL.y !=0 || ptpSys->cell[i][j][k].fL.z!=0){
							glBegin(GL_POINTS); 
							glPointSize(3.0f); 
							glColor3f(0,0,1);
							glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fL.x))/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fL.y))/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fL.z))/MAG);

							glEnd();
							//}

						}
						if(t_debug==3){


							glBegin( GL_LINES );
							glColor3f(1,1,0); 
							glVertex3f(ptpSys->cell[i][j][k].center.x/MAG,ptpSys->cell[i][j][k].center.y/MAG,	ptpSys->cell[i][j][k].center.z/MAG);
							glVertex3f((ptpSys->cell[i][j][k].center.x)/MAG,
								(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fZ.y+ptpSys->cell[i][j][k].fL.y))/MAG,
								(ptpSys->cell[i][j][k].center.z)/MAG);
							glEnd();

							if(ptpSys->cell[i][j][k].fZ.x!=0 || ptpSys->cell[i][j][k].fZ.y !=0 || ptpSys->cell[i][j][k].fZ.z!=0 || ptpSys->cell[i][j][k].fL.x!=0 || ptpSys->cell[i][j][k].fL.y !=0 || ptpSys->cell[i][j][k].fL.z!=0){
								glBegin(GL_POINTS); 
								glPointSize(3.0f); 
								glColor3f(0,1,1);
								glVertex3f(ptpSys->cell[i][j][k].center.x/MAG,
									(ptpSys->cell[i][j][k].center.y-(ptpSys->cell[i][j][k].fZ.y+ptpSys->cell[i][j][k].fL.y))/MAG,
									ptpSys->cell[i][j][k].center.z/MAG);
 
								glEnd();

							}
						}
						if(t_debug==4){


							glBegin( GL_LINES );
							glColor3f(0,1,1); 
							glVertex3f(ptpSys->cell[i][j][k].center.x/MAG,ptpSys->cell[i][j][k].center.y/MAG,	ptpSys->cell[i][j][k].center.z/MAG);
							glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fZ.x+ptpSys->cell[i][j][k].fL.x))/MAG,
								(ptpSys->cell[i][j][k].center.y)/MAG,
								(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fZ.z+ptpSys->cell[i][j][k].fL.z))/MAG);
							glEnd();

							if(ptpSys->cell[i][j][k].fZ.x!=0 || ptpSys->cell[i][j][k].fZ.y !=0 || ptpSys->cell[i][j][k].fZ.z!=0 || ptpSys->cell[i][j][k].fL.x!=0 || ptpSys->cell[i][j][k].fL.y !=0 || ptpSys->cell[i][j][k].fL.z!=0){
								glBegin(GL_POINTS); 
								glPointSize(3.0f); 
								glColor3f(1,1,0);
								glVertex3f((ptpSys->cell[i][j][k].center.x+(ptpSys->cell[i][j][k].fZ.x+ptpSys->cell[i][j][k].fL.x))/MAG,
									(ptpSys->cell[i][j][k].center.y)/MAG,
									(ptpSys->cell[i][j][k].center.z+(ptpSys->cell[i][j][k].fZ.z+ptpSys->cell[i][j][k].fL.z))/MAG);

								glEnd();

							}
						}
#endif

					}
				}

							//	printf("active cell..%d.\n", t_c);
				//CELL
				if(t_c)	{

					r.x = (ptpSys->cell[i][j][k].start.x-ptpSys->cell[i][j][k].end.x)/MAG;
					r.y = (ptpSys->cell[i][j][k].start.y-ptpSys->cell[i][j][k].end.y)/MAG;
					r.z = (ptpSys->cell[i][j][k].start.z-ptpSys->cell[i][j][k].end.z)/MAG;



					if(t_c==2){

						glColor3f(0,ptpSys->cell[i][j][k].CollisionParticleNum/10,0);
						if(i==0 && j==0 && k==0){
							glColor3f(1,0,0);
						}
						if(i==ptpSys->ptp->n.x-1 && j==ptpSys->ptp->n.y-1 && k==ptpSys->ptp->n.z-1){
							glColor3f(0,0,1);
						}

						glTranslatef((ptpSys->cell[i][j][k].start.x/MAG)-(r.x/2),(ptpSys->cell[i][j][k].start.y/MAG)-(r.y/2),(ptpSys->cell[i][j][k].start.z/MAG)-(r.z)/2);
						glScalef(r.x,r.y,r.z);

						glutWireCube(1);				


					}else if(t_c==1){		//just for active cell
						if(ptpSys->cell[i][j][k].CollisionParticleNum){
							glTranslatef((ptpSys->cell[i][j][k].start.x/MAG)-(r.x/2),(ptpSys->cell[i][j][k].start.y/MAG)-(r.y/2),(ptpSys->cell[i][j][k].start.z/MAG)-(r.z)/2);
							glScalef(r.x,r.y,r.z);
							glColor4f(1,0,0,0.5);//glColor3f(0.4,0.4,0.4);
							glutWireCube(1);
							glutSolidCube(1);
							if(ptpSys->cell[i][j][k].fZ.x!=0 || ptpSys->cell[i][j][k].fZ.y !=0 || ptpSys->cell[i][j][k].fZ.z!=0){
								glColor4f(1,1,1,1);//glColor4f(1,0,0,0.5);
								glutSolidCube(1);
								
								//printf("active cell...\n");
							}

						}

					}else if(t_c==3){


					}else if(t_c==4){


					}



					//		}
				}
				glPopMatrix();				
			}
		}
	}
	glPopMatrix();


	glDisable(GL_DEPTH_TEST);

	//Force Display
	if(t_d){
		glPushMatrix();
		glTranslatef(foot.loc.x, foot.loc.y, foot.loc.z);

		glBegin( GL_LINES );
		glColor3f(1,0,0); 
		glVertex3f(0,0,0);  glVertex3f( -forceR.x, -forceR.y, -forceR.z);
		glEnd();

		glPopMatrix();

	}
	//Velocity Display
	if(t_v){
		glPushMatrix();
		glTranslatef(foot.loc.x, foot.loc.y, foot.loc.z);

		glBegin( GL_LINES );
		glColor3f(0,0,1); 
		glVertex3f(0,0,0);  glVertex3f( foot.vel.x, foot.vel.y, foot.vel.z);
		glEnd();

		glPopMatrix();
	}

	glDisable(GL_DEPTH_TEST);
	//x,y,z axis
	if(t_x){
		glPushMatrix();
		glLineWidth(3.0f);
		glBegin( GL_LINES );

		glColor3f( 1.0, 0.0, 0.0 ); 
		glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 10, 0.0, 0.0 ); /* X axis      */

		glColor3f( 0.0, 1.0, 0.0 );
		glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 0.0, 10, 0.0 ); /* Y axis      */

		glColor3f( 0.0, 0.0, 1.0 );
		glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 0.0, 0.0, 10 ); /* Z axis  --left hand coordination system  */ 		

		glEnd();
		glLineWidth(1.0f);
		glPopMatrix();
	}

	// Text Display
	//	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	if (Projection == 1) {
		glTranslatef(-1,0.5,-1.5);
	}
	if (Projection == 2) {
		glTranslatef(-0.1,-0.1,-1.5);
	}

	glColor3f(0.0,0.0,0.0); // output the string, in red
	snprintf(msg,sizeof(msg),"%.2f[s] STEP:%d (%f) // TOTAL %d Particles:(%d*%d*%d) AREA(%.2f,%.2f,%.2f)mm",msec/100,step,step_size,ptpSys->ptp->n.x*ptpSys->ptp->n.y*ptpSys->ptp->n.z,ptpSys->ptp->n.x,ptpSys->ptp->n.y,ptpSys->ptp->n.z,ptpSys->ptp->end.x-ptpSys->ptp->start.x,ptpSys->ptp->end.y-ptpSys->ptp->start.y,ptpSys->ptp->end.z-ptpSys->ptp->start.z);

	glColor3f(0.0,0.4,0.0); // output the string, in red
	out_string_at( GLUT_BITMAP_9_BY_15, -0.9f, 0.9f, 0.f, msg );
	snprintf(msg,sizeof(msg),"FOOT:(%.2f,%.2f,%.2f)mm FOOTv(%.2f,%.2f,%.2f)mm/s",foot.loc.x,foot.loc.y,foot.loc.z,foot.vel.x,foot.vel.y,foot.vel.z);

	out_string_at( GLUT_BITMAP_9_BY_15, -0.9f, 0.85f, 0.f, msg );
	glColor3f(1.0,0.0,0.0); // output the string, in red
	snprintf(msg,sizeof(msg),"FORCE(%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f)",forceR.x,forceR.y,forceR.z,forceT.x,forceT.y,forceT.z);

	out_string_at( GLUT_BITMAP_9_BY_15, -0.9f, 0.8f, 0.f, msg );
	glPopMatrix();

	glPopMatrix();
	glutSwapBuffers();
	glFlush();

}

void printFOOT(){
	printf("FOOT:cm(%.2f,%.2f,%.2f)mm v(%.1f,%.1f,%.1f)mm/s r:%.2fmm l:%.2fmm t:%d\n",foot.loc.x,foot.loc.y,foot.loc.z,foot.vel.x,foot.vel.y,foot.vel.z,foot.r,foot.l, foot.type);
}

void makeOpenGLView(int argc, char **argv, FOOT f){
	glutInit(&argc, argv);
	GLint SubMenu = glutCreateMenu(DoMenu);
	time_t timer;
	struct tm *t;

	foot = f;

	glutInitWindowSize(800,600);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("PTPSystem Viewer");

	glutAddMenuEntry("Orthographic",10);
	glutAddMenuEntry("Frustrum",11);
	glutAddMenuEntry("Perspective",12);
	glutAddMenuEntry("Blending",13);
	glutAddMenuEntry("Anti-Aliasing",14);

	glutCreateMenu(DoMenu);
	glutAddMenuEntry("Foot",1);
#ifdef USE_SOIL_VISUALIZATION
	glutAddMenuEntry("Soil",7);
#endif
	glutAddMenuEntry("Particle",2);
	glutAddMenuEntry("Cell",3);
	glutAddMenuEntry("Axis",4);
	glutAddMenuEntry("Force",5);
	glutAddMenuEntry("Movement",6);
	glutAddMenuEntry("Velocity",8);
#if defined USE_FORCE_DEBUG
	glutAddMenuEntry("Debug",20);
#endif


	glutAddSubMenu("View Type",SubMenu);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glutDisplayFunc(DoDisplay);
	glutKeyboardFunc(DoKeyboard);
	glutSpecialFunc(DoSpecial);
	glutReshapeFunc(DoReshape);
	glutMouseFunc(DoMouse);
	glutMotionFunc(DoMouseMotion);


	glEnable(GL_CULL_FACE); 




	timer = time(NULL); 
	t = localtime(&timer); 	
	snprintf(filename,sizeof(filename),"result%d_%d_%d-%d_%d_%d.csv", t->tm_year + 1900,t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
	snprintf(filename_cell,sizeof(filename),"result%d_%d_%d-%d_%d_%d_cell.csv", t->tm_year + 1900,t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
	out = fopen(filename, "w");
	fprintf(out,"msec,force_x,force_y,force_z,force_T_x,force_T_y,force_T_z,foot_loc_x,foot_loc_y,foot_loc_z\n");
	fclose(out); 
	/*
//	fopen_s(&out_cell,filename_cell, "w");
//	fprintf(out,"msec,x,y,z,particleNum\n");
//	fclose(out); 
	*/
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	glutMainLoop();	
}

void test(){
	vecN num;
	vecD start;
	vecD end;
	foot.mass = 0.456;
#ifdef SPHERE_EXPERIMENT
	//Number of Cells
	num.x = 50; 
	num.y = 50; 
	num.z = 25; 

	start.x = -50; //  mm
	start.y = -50;
	start.z = -50;

	end.x = 50;
	end.y = 50;    
	end.z = 0;   

	//Initialize
	foot.r=25;    //mm
	foot.l=0;
	foot.d=0;

	foot.loc.x =0;
	foot.loc.y =0;
	foot.loc.z =25.1;

	foot.type = SPHERE;
#endif


#ifdef SPHERE_EXPERIMENT2
	//Number of Cells
	num.x = 20;
	num.y = 20;
	num.z = 20;

	foot.mass = 0;

	start.x = -30;
	start.y = -60;
	start.z = -30;

	end.x = 30;
	end.y = 0;
	end.z = 30;

	//Initialize
	foot.r=25;
	foot.l=0;
	foot.d=0;

	foot.loc.x =0;
	foot.loc.y =26;
	foot.loc.z =0;
	

	foot.type = SPHERE; //*/
	//foot.type = CYLINDER  
#endif

#ifdef CYLINDER_EXPERIMENT
	foot.mass = 0;
	num.x = 11;  //number of cells = num - 1 
	num.y = 11;
	num.z = 11;

	start.x = -30;
	start.y = -60;
	start.z = -30;

	end.x = 30;
	end.y = 0;
	end.z = 30;

	//Initialize
	foot.r=25;
	foot.l=10;
	foot.d=0;

	foot.loc.x =0;
	foot.loc.y =5.1;
	foot.loc.z =0;

	foot.type = CYLINDER; //*/

#endif


#ifdef CELL_TEST_SPHERE
	/// For Test
	num.x = 1;
	num.y = 1;
	num.z = 1;

	start.x = -5;
	start.y = -10;
	start.z = -5;

	end.x = 5;
	end.y = 0;
	end.z = 5;

	foot.r=25 ;
	foot.l=0;
	foot.d=0;

	foot.loc.x =0;
	foot.loc.y =30;
	foot.loc.z =0;

	foot.type = SPHERE; //*/

#endif
#ifdef CELL_TEST_CYLINDER
	///For Test(Cylinder)
	num.x = 3;
	num.y = 3;
	num.z = 3;

	start.x = -25;
	start.y = -15;
	start.z = -25;

	end.x = 25;
	end.y = 0;
	end.z = 25;

	foot.r=25;
	foot.l=10;
	foot.d=0;

	foot.loc.x =0;
	foot.loc.y =6;
	foot.loc.z =0;

	foot.type = CYLINDER; //*/

#endif

	ptpSys->initializePTP(&num,&start,&end);
}
} //end of namespace soildynamics

using namespace soildynamics;

int main(int argc, char **argv)
{
	test();
	PTPopenGL();
	makeOpenGLView(argc, argv, foot);
	return 1;
}

