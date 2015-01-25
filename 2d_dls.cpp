#include <math.h>
#include <stdio.h>

#include <glut.h>

#define PI     3.14159265358979323846
#define HOR  500	/* screen resolution */
#define VER  500
#define DEADBAND  20	/* zero end effector velocity region */
#define DELTA_T  500	/* computation cycle time */
#define K_P  0.5	/* position error feedback gain */
#define MAXVELNORM  10	/* maximum end effector velocity norm */
#define LAMBDA2  625	/* damping factor squared */
#define MAXDOF  10	/* maximum number of degrees of freedom for robot */
#define MAXDOFPLUS  MAXDOF+1 /* maximum number of robot coordinate frames */

typedef struct
{
        float x;
        float y;
} vector; 


/* GLOBALS */

	/* file pointers */
	FILE *fopen(), *fp_arm;
 

	/* arm parameters */
	int nmbrdof;		/* number of degrees of freedom */
	float lngth[MAXDOF];	/* link lengths */
	float theta[MAXDOF];	/* joint angle positions */
	float thetadot[MAXDOF]; /* joint angle velocities */
	vector link_frame[MAXDOFPLUS]; /* world position (x,y) of individual */
				       /* link coordinate frames and end effector */

	vector efjacob[MAXDOF]; /* Jacobian for end effector */

	vector x_desired;	/* desired end effector trajectory position */
	vector xdot_desired;	/* desired end effector trajectory velocity */
	vector xdot_commanded;	/* commanded end effector velocity */

	vector arm[MAXDOF];	/* screen positions for displaying */
				/* the robot arm */
    
 defnarm() /* read in description of robot arm */
{
	int i;

	fscanf(fp_arm,"%d",&nmbrdof);
	if (nmbrdof > MAXDOF) printf("ERROR: TOO MANY DEGREES OF FREEDOM IN ROBOT\n");
	printf("%%\tNumber of Links: %d\n",nmbrdof);
	printf("%%\tLink Lengths \tAngles (degrees)\n");

	for (i = 0; i < nmbrdof; i++)
	{
		fscanf(fp_arm,"%f%f",&lngth[i],&theta[i]);
		printf("%%\t%f\t%f\n",lngth[i],theta[i]);
		theta[i]=theta[i]*PI/180; /* convert to radians */
	}
}

efvel() /* calculate commanded end effector velocity */
{
	float velnorm;

	/* integrate desired end effector velocity */
	/* to obtain desired end effector position */
	x_desired.x=x_desired.x+xdot_desired.x;
	x_desired.y=x_desired.y+xdot_desired.y;

	/* commanded end effector velocity is a combination of */
	/* the desired velocity plus a positon error term */
	xdot_commanded.x=xdot_desired.x+K_P*(x_desired.x-link_frame[nmbrdof].x);
	xdot_commanded.y=xdot_desired.y+K_P*(x_desired.y-link_frame[nmbrdof].y);

	/* if commanded velocity is excessive, then scale down to maximum allowable */
	velnorm=sqrt(xdot_commanded.x*xdot_commanded.x+xdot_commanded.y*xdot_commanded.y);
	if (velnorm>MAXVELNORM)
	{
		xdot_commanded.x=xdot_commanded.x*MAXVELNORM/velnorm;
		xdot_commanded.y=xdot_commanded.y*MAXVELNORM/velnorm;
	}
}

efjacobian() /* Calculate end effector Jacobian */
{
	int i,j;
	float psi; /* absolute joint angle */
	vector aj[MAXDOF]; /* absolute angle jacobian */

	/* calculate terms of Jacobian in absolute angles */
	psi=0;
	for (i = 0; i < nmbrdof; i++)
	{
		psi=psi+theta[i];
		aj[i].x=-lngth[i]*sin(psi);
		aj[i].y= lngth[i]*cos(psi);
	}
	
	/* Jacobian in relative angles is a sum of absolute terms */
	for (i = 0; i < nmbrdof; i++)
	{
		efjacob[i].x=0;
		efjacob[i].y=0;
		for (j = i; j < nmbrdof; j++)
		{
			efjacob[i].x=efjacob[i].x+aj[j].x;
			efjacob[i].y=efjacob[i].y+aj[j].y;
		}
	}
}


solve() /* solve the inverse kinematics using damped least squares */
{
	int i;
	float determ,temp;
	vector efpseudo[MAXDOF];
	float jjT[2][2];

	/* calculate the Jacobian times the Jacobian transpose (jjT) */
	jjT[0][0]=0;
	jjT[1][0]=0;
	jjT[0][1]=0;
	jjT[1][1]=0;
	for (i = 0; i < nmbrdof; i++)
	{
		jjT[0][0]=jjT[0][0]+efjacob[i].x*efjacob[i].x;
		jjT[1][0]=jjT[1][0]+efjacob[i].y*efjacob[i].x;

		jjT[0][1]=jjT[0][1]+efjacob[i].x*efjacob[i].y;
		jjT[1][1]=jjT[1][1]+efjacob[i].y*efjacob[i].y;
	}

	/* add damping factor to the diagonal */
	jjT[0][0]=jjT[0][0]+LAMBDA2;
	jjT[1][1]=jjT[1][1]+LAMBDA2;

	/* calculate the inverse of jjT */
	determ=jjT[0][0]*jjT[1][1]-jjT[0][1]*jjT[1][0];
	temp=jjT[0][0];
	jjT[0][0]=jjT[1][1]/determ;
	jjT[0][1]=-jjT[0][1]/determ;
	jjT[1][0]=-jjT[1][0]/determ;
	jjT[1][1]=temp/determ;

	/* multiply by jT for damped least squares inverse, i.e. jT(jjT)^-1 */
	for (i = 0; i < nmbrdof; i++)
	{
		efpseudo[i].x=0;
		efpseudo[i].y=0;
		efpseudo[i].x=efpseudo[i].x+efjacob[i].x*jjT[0][0]
					   +efjacob[i].y*jjT[1][0];
		efpseudo[i].y=efpseudo[i].y+efjacob[i].x*jjT[0][1]
					   +efjacob[i].y*jjT[1][1];
	}

	/* multiply Jacobian inverse times the commanded end effector */
	/* velocity to obtain the desired joint angle velocity */
	for (i = 0; i < nmbrdof; i++)
	thetadot[i]=efpseudo[i].x*xdot_commanded.x+
		    efpseudo[i].y*xdot_commanded.y;

	/* integrate the joint velocity to find joint position */
	for (i = 0; i < nmbrdof; i++)
	theta[i]=theta[i]+thetadot[i];
	
}


dir_kin() /* calculate the direct (forward) kinematics of the robot */
{
	int i;
	float psi; /* absolute joint angle (sum of relative) */


	psi=0;

	arm[0].x=0;
	arm[0].y=0;

	link_frame[0].x=0;
	link_frame[0].y=0;
	
	/* calculate the position of all link coordinate frames */
	for (i = 0; i < nmbrdof; i++)
	{
		psi=psi+theta[i];
		link_frame[i+1].x=link_frame[i].x+lngth[i]*cos(psi);
		link_frame[i+1].y=link_frame[i].y+lngth[i]*sin(psi);
	}

	/* transform robot arm to screen coordinates (y axis pointing down) */
	for (i = 0; i < nmbrdof+1; i++) 
	{
		arm[i].x = (int) link_frame[i].x+HOR/2;
		arm[i].y = (int) link_frame[i].y+VER/2;
	}
}


void read_mouse(int x, int y)
{
	vector delta;
			
	/* transform mouse position to world coordinate frame */
	delta.x =  x-3*HOR/2;
    delta.y = -y+VER/2;

	/* implement deadband region */
	if (delta.x < DEADBAND && delta.x > -DEADBAND ) delta.x=0;
	if (delta.y < DEADBAND && delta.y > -DEADBAND ) delta.y=0;

	/* calculate the required average velocity over the computation interval */
	xdot_desired.x= ((float)delta.x)/DELTA_T;
	xdot_desired.y= ((float)delta.y)/DELTA_T;

}

main(argc, argv)
int argc;
char *argv[];
{
	init_windows(argc,argv);

	fp_arm = fopen("arm", "r");

	defnarm(); /* read in arm description */

	/* initialize desired end effector position and velocity */
	dir_kin();
	x_desired.x=link_frame[nmbrdof].x;
	x_desired.y=link_frame[nmbrdof].y;
	xdot_desired.x=0;
	xdot_desired.y=0;

	glutDisplayFunc(display);
	glutMotionFunc(read_mouse);
	glutMainLoop();
	return(0);
}
