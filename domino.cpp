/*
 * Dominó.cpp
 *
 * Copyright 2014 Rafael Andrade <rafaelsfg@hotmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Programa que simula a queda de uma fila de dominós
 *
 * Atualizado em 18/12/2015
 *
 * Teclas:
 * * 'p' para derrubar os dominós
 * * 'a' mover para esquerda
 * * 'd' mover para direita
 * * 's' mover para trás
 * * 'w' moer para frente
 * * 'ESC' para sair
 */

#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
#include <ode/ode.h>

#define MAX_CONTATO   20          // Número máximo de contatos
#define           G   9.81      // Define a gravidade em m/s^2
#define grau          57.295779506
#define PASSO_TEMPO   10       // Define o Passo de Tempo
#define VEL_GIRO      0.002   // Velocidade do giro da câmera
#define NUM_DOMINO    60    // Número de dominós

float obsteta = 2.79, obsfi = 1.5, distancia = 1.0;  //  Angulos de visualização do observador
unsigned int dT;    // Diferença de tempo entre os frames

dWorldID Mundo;
dSpaceID Espaco;
dBodyID domino[NUM_DOMINO];      //  Vetor que contém os Dominós
dGeomID plano;                  //  Plano
dJointGroupID GrupoContato;


const dReal stepSize = PASSO_TEMPO / 1000.0;      //  Passo de tempo fixo

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função que Inicializa o ODE
//
///////////////////////////////////////////////////////////////////////////////////////////

void InitODE(void)
{
    dInitODE();
    Mundo = dWorldCreate();
    dWorldSetGravity(Mundo, 0.0, 0.0, -G);

    dWorldSetERP(Mundo, 0.5);
    dWorldSetCFM(Mundo, 1E-5);
    dWorldSetContactMaxCorrectingVel(Mundo, 1.0E2);
    dWorldSetContactSurfaceLayer(Mundo, 0.0001);
    dWorldSetDamping(Mundo, 0.0001, 0.0001);

    dWorldSetAutoDisableFlag(Mundo, 1); // Ativa a auto-desabilitação

    dWorldSetAutoDisableLinearThreshold(Mundo, 0.5); // Velocidade linear mínima

    dWorldSetAutoDisableAngularThreshold(Mundo, 2.0); // Velocidade angular mínima

    dWorldSetAutoDisableSteps(Mundo, 50);  // Número de passos para desabilitar

    Espaco = dHashSpaceCreate(0);


    ////////////////  Criar Objetos  ////////////////////////////////////////

    dMass massa;
    dMassSetBoxTotal (&massa, 0.01, 0.004, 0.02, 0.05);

    // Cria os dominós
    for(int i = 0; i < NUM_DOMINO; i++)
    {
        domino[i] = dBodyCreate(Mundo);
        dBodySetMass(domino[i], &massa);
        dGeomSetBody(dCreateBox(Espaco, 0.004, 0.02, 0.05), domino[i]);
        dBodySetPosition(domino[i], -0.4+(double)i * 0.02, 0.0, 0.025);
    }

    // Cria o chao
    plano = dCreatePlane(Espaco,0.0,0.0,1.0,0.0);

    //////////////////////////////////////////////////////////////////////////

    // Cria os contatos
    GrupoContato = dJointGroupCreate (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função que detecta Colisões
//
///////////////////////////////////////////////////////////////////////////////////////////

void Colisao(void *teste, dGeomID o1, dGeomID o2)
{
    int i, NColisao = 0;
    dBodyID Corpo1, Corpo2;
    dContact contact[MAX_CONTATO];

    NColisao = dCollide(o1, o2, MAX_CONTATO, &contact[0].geom, sizeof(dContact));

    if(NColisao != 0)
    {
        Corpo1 = dGeomGetBody(o1);
        Corpo2 = dGeomGetBody(o2);

        for ( i = 0; i < NColisao; i++)
        {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.slip1 = 10.0;
            contact[i].surface.slip2 = 10.0;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.2;

            dJointID c = dJointCreateContact(Mundo, GrupoContato, contact+i);
            dJointAttach(c, Corpo1, Corpo2);
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função que desenha na Tela
//
///////////////////////////////////////////////////////////////////////////////////////////

void Caixa( double l, double c, double a )
{
    l = l / 2.0;
    c = c / 2.0;
    a = a / 2.0;

    glBegin(GL_QUADS);
        glNormal3f(0.0, 0.0, -1.0);
        glVertex3f(-l, -c, -a);
        glVertex3f(l, -c, -a);
        glVertex3f(l, c, -a);
        glVertex3f(-l, c, -a);
    glEnd();

    glBegin(GL_QUADS);
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3f(-l, -c, a);
        glVertex3f(l, -c, a);
        glVertex3f(l, c, a);
        glVertex3f(-l, c, a);
    glEnd();

    glBegin(GL_QUADS);
        glNormal3f(-1.0, 0.0, 0.0);
        glVertex3f(-l, -c, -a);
        glVertex3f(-l, -c, a);
        glVertex3f(-l, c, a);
        glVertex3f(-l, c, -a);
    glEnd();

    glBegin(GL_QUADS);
        glNormal3f(1.0, 0.0, 0.0);
        glVertex3f(l, -c, -a);
        glVertex3f(l, -c, a);
        glVertex3f(l, c, a);
        glVertex3f(l, c, -a);
    glEnd();

    glBegin(GL_QUADS);
        glNormal3f(0.0, -1.0, 0.0);
        glVertex3f(-l, -c, -a);
        glVertex3f(-l, -c, a);
        glVertex3f(l, -c, a);
        glVertex3f(l, -c, -a);
    glEnd();

    glBegin(GL_QUADS);
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(-l, c, -a);
        glVertex3f(-l, c, a);
        glVertex3f(l, c, a);
        glVertex3f(l, c, -a);
    glEnd();
}

void Desenhar(void)
{
    double obsx, obsy, obsz;

    // Calcula a posição do observador
    obsx = distancia * sin(obsfi) * cos(obsteta);
    obsy = distancia * sin(obsfi) * sin(obsteta);
    obsz = distancia * cos(obsfi);

    // Posiciona a Câmera
    glLoadIdentity();
    gluLookAt(obsx, obsy, obsz, 0.0, 0.0, 0.0, 0.0f,0.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    ///////////////////////////////////////////////////////////////////////////////////////////
    //  Área de Desenho
    ///////////////////////////////////////////////////////////////////////////////////////////

    const dReal *pos, *q;
    dReal angulo, s, x1 = 0.0, x2 = 0.0, x3 = 0.0;

    GLfloat posicao_luz[] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv( GL_LIGHT0, GL_POSITION, posicao_luz );
    glEnable( GL_LIGHTING );

    /// Desenha os dominós
    for( int i = 0; i < NUM_DOMINO; i++ )
    {
        pos = dBodyGetPosition( domino[i] );
        glPushMatrix();
        glTranslatef ( pos[0], pos[1], pos[2] );
        q = dBodyGetQuaternion( domino[i] );
        angulo = 2.0 * acos( q[0] );
        if( angulo != 0.0 )
        {
            s = sin( angulo / 2.0 );
            x1 = q[1] / s;
            x2 = q[2] / s;
            x3 = q[3] / s;
        }
        glRotatef( angulo * grau, x1, x2, x3 );
        glColor3f( 0.0, 0.0, 0.7 );
        Caixa( 0.004, 0.02, 0.05 );
        glPopMatrix();
    }
    /// Desenha o plano
    glColor3f( 1.0, 1.0, 1.0 );
    glBegin( GL_QUADS );
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3f( 10.0, 10.0, 0.0 );
        glVertex3f( -10.0, 10.0, 0.0 );
        glVertex3f( -10.0, -10.0, 0.0 );
        glVertex3f( 10.0, -10.0, 0.0 );
    glEnd();

    ///////////////////////////////////////////////////////////////////////////////////////

    glPopMatrix();
    glutSwapBuffers();
}


///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função de Iluminação do Ambiente
//
///////////////////////////////////////////////////////////////////////////////////////////

void Iluminacao(void)
{
    GLfloat ambiente[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat difusa[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat especular[] = { 1.0, 1.0, 1.0, 1.0 };

    glEnable(GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambiente);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, difusa);
    glLightfv(GL_LIGHT0, GL_SPECULAR, especular);

    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função usada para Redimensionar a Janela
//
///////////////////////////////////////////////////////////////////////////////////////////

void Ajustedimensao(GLsizei w, GLsizei h)
{
    if(h == 0)
        h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)w/(GLfloat)h, 0.01, 800);
    glMatrixMode(GL_MODELVIEW);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função para mudar a visualização do espaço pelo movimento do mouse
//
///////////////////////////////////////////////////////////////////////////////////////////

void MoveMouse(int x, int y)
{
    float passo = VEL_GIRO * dT;
    float r = 0.0f, q = 0.0f;
    static int Xo = 0, Yo = 0;

    if(x > Xo) r = -passo;
    if(x < Xo) r = passo;
    if(y > Yo && obsfi > 0.02) q = -passo;
    if(y < Yo && obsfi < 3.13) q = passo;

    obsteta += r;
    obsfi += q;

    Xo = x;
    Yo = y;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função de animação da Simulação
//
///////////////////////////////////////////////////////////////////////////////////////////

void Timer(int w)
{
    // Calcula a diferença de tempo entre os frames
    static unsigned int t0 = glutGet(GLUT_ELAPSED_TIME);

    unsigned int t1 = glutGet(GLUT_ELAPSED_TIME);

    dT = t1 - t0;

    t0 = t1;
    /////////

    dSpaceCollide (Espaco, 0, &Colisao);   //  Verifica as Colisões

    dWorldQuickStep(Mundo, stepSize);      //  Atualiza o Tempo e os Objetos

    dJointGroupEmpty (GrupoContato);

    glutPostRedisplay();

    glutTimerFunc(PASSO_TEMPO, Timer, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função recebe as informações do teclado
//
///////////////////////////////////////////////////////////////////////////////////////////

void Teclado(unsigned char key, int a, int b)
{
    switch (key)
    {
        case 27: // Fecha o programa
                dCloseODE();
                exit(0);
                break;

        case '+': // Aumenta o Zoom
                distancia *= 1.5;
                break;

        case '-': // Diminue o Zoom
                distancia /= 1.5;
                break;

        case 'p': // Inicia a queda
                if(dBodyIsEnabled(domino[0]) == 0)
                    dBodyEnable(domino[0]);
                dBodySetForce(domino[0], 0.08, 0.0, 0.0);

                break;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função Principal
//
///////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char*argv[])
{
    glutInit (&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(700,500);
    glutInitWindowPosition(50,50);
    glutCreateWindow("Queda de dominos");
    glutDisplayFunc(Desenhar);
    glutReshapeFunc(Ajustedimensao);
    glutMotionFunc(MoveMouse);
    glutKeyboardFunc(Teclado);
    Iluminacao();
    InitODE();
    Timer(1);
//  glutFullScreen ( );
    glutMainLoop();
}
