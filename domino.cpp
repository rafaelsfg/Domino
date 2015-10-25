/*
 * Dominó.cpp
 *
 * Copyright 2014 Rafael Andrade <rafaelsandrade@gmail.com>
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
#define           G   980.0      // Define a gravidade em cm/s^2
#define grau          57.295779506
#define PASSO_TEMPO   10            // Define o Passo de Tempo

#define NUM_DOMINO    60

GLfloat obsteta = 20.0, obsfi = 20.0;  //  Angulos de visualização do observador
GLfloat d = 2.0;                   //  Distância do observador à origem
GLfloat posx = 0.0, posy = 0.0, posz = 0.0;   //  Posição do observador

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
    dWorldSetERP(Mundo, 0.7);
    Espaco = dHashSpaceCreate(0);
    //dWorldSetContactMaxCorrectingVel(Mundo, 1000.0);
    //dWorldSetContactSurfaceLayer(Mundo, 0.01);

    ////////////////  Criar Objetos  ////////////////////////////////////////

    dMass massa;
    dMassSetBoxTotal (&massa, 0.01, 0.4, 2.0, 5.0);

    for(int i = 0; i < NUM_DOMINO; i++)
    {
        domino[i] = dBodyCreate(Mundo);
        dBodySetMass(domino[i], &massa);
        dGeomSetBody(dCreateBox(Espaco, 0.4, 2.0, 5.0), domino[i]);
        dBodySetLinearDamping(domino[i], 0.05);
    }

    dWorldSetAutoDisableFlag(Mundo, 1);
    dWorldSetAutoDisableLinearThreshold(Mundo, 0.1);

    // Cria o chao
    plano = dCreatePlane(Espaco,0.0,0.0,1.0,0.0);

    //////////////////////////////////////////////////////////////////////////
    // Cria os contatos
    GrupoContato = dJointGroupCreate (0);

    for(int i = 0; i < NUM_DOMINO; i++)
    {
        dBodySetPosition(domino[i], -40+(double)i * 2.0, 0.0, 2.5);
    }
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
            contact[i].surface.mu = 1.0E1;//dInfinity;
            contact[i].surface.slip1 = 0.9;
            contact[i].surface.slip2 = 0.9;
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

void Caixa( double x, double y, double z )
{
    glBegin(GL_QUADS);
    glVertex3f(-x / 2.0, -y / 2.0, -z / 2.0);
    glVertex3d(x / 2.0, -y / 2.0, -z / 2.0);
    glVertex3f(x / 2.0, y / 2.0, -z / 2.0);
    glVertex3f(-x / 2.0, y / 2.0, -z / 2.0);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-x / 2.0, -y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, -y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, y / 2.0, z / 2.0);
    glVertex3f(-x / 2.0, y / 2.0, z / 2.0);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-x / 2.0, -y / 2.0, -z / 2.0);
    glVertex3f(-x / 2.0, -y / 2.0, z / 2.0);
    glVertex3f(-x / 2.0, y / 2.0, z / 2.0);
    glVertex3f(-x / 2.0, y / 2.0, -z / 2.0);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(x / 2.0, -y / 2.0, -z / 2.0);
    glVertex3f(x / 2.0, -y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, y / 2.0, -z / 2.0);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-x / 2.0, -y / 2.0, -z / 2.0);
    glVertex3f(-x / 2.0, -y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, -y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, -y / 2.0, -z / 2.0);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-x / 2.0, y / 2.0, -z / 2.0);
    glVertex3f(-x / 2.0, y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, y / 2.0, z / 2.0);
    glVertex3f(x / 2.0, y / 2.0, -z / 2.0);
    glEnd();
}

void Desenhar(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //  Movimento do Espaço
    ///////////////////////////////////////////////////////////////////////////////////////////

    glPushMatrix();
    glScalef(d, d, d);
    glTranslatef(posx, posy, posz);
    glRotatef(obsfi,-1,0,0);
    glRotatef(obsteta,0,0,1);


    ///////////////////////////////////////////////////////////////////////////////////////////
    //  Área de Desenho
    ///////////////////////////////////////////////////////////////////////////////////////////

    const dReal *pos, *q;
    dReal angulo, s, x1 = 0.0, x2 = 0.0, x3 = 0.0;

    GLfloat posicao_luz[] = { 50.0, 50.0, 100.0, 1.0 };
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
        Caixa( 0.4, 2.0, 5.0 );
        glPopMatrix();
    }
    /// Desenha o plano
    glColor3f( 1.0, 1.0, 1.0 );
    glBegin( GL_QUADS );
    glVertex3f( 100.0, 100.0, 0.0 );
    glVertex3f( -100.0, 100.0, 0.0 );
    glVertex3f( -100.0, -100.0, 0.0 );
    glVertex3f( 100.0, -100.0, 0.0 );
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
    GLfloat ambiente[] = { 0.0, 0.0, 0.0, 1.0 };
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
//  Função usada para especificar o volume de visualização
//
///////////////////////////////////////////////////////////////////////////////////////////

void Visualizacao(GLfloat fAspect)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0,fAspect,0.1,800);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,100,0, 0,0,0, 0,0,1);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função usada para Redimensionar a Janela
//
///////////////////////////////////////////////////////////////////////////////////////////

void Ajustedimensao(GLsizei w, GLsizei h)
{
    GLfloat fAspect;
    if ( h == 0 ) h = 1;
    glViewport(0, 0, w, h);
    fAspect = (GLfloat)w/(GLfloat)h;
    Visualizacao(fAspect);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função para mudar a visualização do espaço pelo movimento do mouse
//
///////////////////////////////////////////////////////////////////////////////////////////

void MoveMouse(int x, int y)
{
    static int Xo=0, Yo=0;
    double r=0.0,q=0.0;

    if(x>Xo) r= 2.0;
    if(x<Xo) r=-2.0;
    if(y>Yo) q=2.0;
    if(y<Yo) q=-2.0;

    obsteta=obsteta+r;
    obsfi=obsfi+q;

    Xo=x;
    Yo=y;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//  Função de animação da Simulação
//
///////////////////////////////////////////////////////////////////////////////////////////

void Timer(int w)
{
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
                d*=1.5;
                break;

        case '-': // Diminue o Zoom
                d/=1.5;
                break;

        case 'p': // Inicia a queda
                dBodySetForce(domino[0], 8.0, 0.0, 0.0);
                break;

        case 'a': // Ir para esquerda
                posx -= 1.0;
                break;

        case 'd':  // Ir para direita
                posx += 1.0;
                break;

        case 'w':  // Ir para frente
                posy += 1.0;
                break;

        case 's':  // Ir para trás
                posy -= 1.0;
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
