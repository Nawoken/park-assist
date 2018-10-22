import numpy as np
import math 
import matplotlib.pyplot as plt


#calcule la norme du vecteur MN à 2 composantes

def norme(M,N):
    L=[]
    for i in range(0, 2):
        L.append(N[i]-M[i])
    return(math.sqrt(L[0]**2+L[1]**2))
    
    
#définit la place de parking (BCDO) pour la tracer plus tard

def Tracé_place(O=[0,0],B=[0.385,0],C=[0.385,0.154],D=[0,0.154]):
    Place_x=[B[0],C[0],D[0],O[0]]
    Place_y=[B[1],C[1],D[1],O[1]]
    return(Place_x,Place_y)


#donne la projection sur (x0,y0) d'un vecteur dans la base (x1,y1)

def projection(L,theta,G):
    P=[0,0]
    P[0]=L[0]*np.cos(theta)-L[1]*np.sin(theta)+G[0]
    P[1]=L[0]*np.sin(theta)+L[1]*np.cos(theta)+G[1]
    return(P)

#calcule la trajectoire des points G, A1, A2, A3, A4

#a = largeur de la voiture
#b = longueur de la voiture
#L_1 = longueur de la place
#L_2 = largeur de la place
#h = distance de sécurité avec le trottoir et les voitures, à maintenir pendant toute la manoeuvre
#G_x0 = position initiale du centre de la voiture selon l'axe x
#G_y0 = position initiale du centre de la voiture selon l'axe y, supérieur à h+a/2
#e: erreur sur le positionnement

def auto_park(a,b,L_1,L_2,h,G_x0,G_y0,e):
        
    #définition de la position des points O,B,C,D (place de parking), et P (moitié de la place+h) dans le repère (O,x0,y0)
    O=[0,0]
    B=[L_1,0]
    C=[L_1,L_2]
    D=[0,L_2]
    P=[h,L_2/2]
     
    #initialisation de la position de G et des points A1, A2, A3, A4 (voiture) dans le repère (O,x0,y0)
    G=[G_x0,G_y0]
    A1=projection([-b/2,a/2],0,G)
    A2=projection([-b/2,-a/2],0,G)
    A3=projection([b/2,-a/2],0,G)
    A4=projection([b/2,a/2],0,G)
    
    #initialisation des listes de coordonnées de G et A1, A2, A3, A4 au cours du temps
    G_x=[G_x0]
    G_y=[G_y0]
    A1_x=[A1[0]]
    A1_y=[A1[1]]
    A2_x=[A2[0]]
    A2_y=[A2[1]]
    A3_x=[A3[0]]
    A3_y=[A3[1]]   
    A4_x=[A4[0]]
    A4_y=[A4[1]]
    A2_proj=projection([-b/2,-a/2],-np.pi/4,G)
    A3_proj=projection([b/2,-a/2],-np.pi/4,G)


    #1ère étape : on recule en ligne droite
    
    while ((A2_proj[0]-P[0])*(A3_proj[1]-A2_proj[1])-(A2_proj[1]-P[1])*(A3_proj[0]-A2_proj[0]))<0: #tant que le point A2 n'est pas à 45° du point P
        G[0]=G[0]-0.01 #la voiture recule de 1 cm
        G_x.append(G[0])
        G_y.append(G[1])
      
        A1=projection([-b/2,a/2],0,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])

        A2=projection([-b/2,-a/2],0,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])   
             
        A3=projection([b/2,-a/2],0,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])  
                
        A4=projection([b/2,a/2],0,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
        
        A2_proj=projection([-b/2,-a/2],-np.pi/4,G)#position imaginaire de A2 si la voiture tournait de 45°
        A3_proj=projection([b/2,-a/2],-np.pi/4,G)#position imaginaire de A3 si la voiture tournait de 45°
        
        
    #2ème étape : on pivote sur place

    A1=projection([-b/2,a/2],-np.pi/4,G)
    A1_x.append(A1[0])
    A1_y.append(A1[1])
    
    A2=projection([-b/2,-a/2],-np.pi/4,G)
    A2_x.append(A2[0])
    A2_y.append(A2[1])
      
    A3=projection([b/2,-a/2],-np.pi/4,G)
    A3_x.append(A3[0])
    A3_y.append(A3[1])
    
    A4=projection([b/2,a/2],-np.pi/4,G)
    A4_x.append(A4[0])
    A4_y.append(A4[1])
    
    G=projection([0,0],-np.pi/4,G)
    G_x.append(G[0])
    G_y.append(G[1])
    
    #3ème étape : on recule jusqu'à ce que A1 soit à la limite de la distance de sécurité verticale
    while A1[1]<L_2-h:
        G[0]=G[0]-0.01
        G[1]=G[1]+0.01
        G_x.append(G[0])
        G_y.append(G[1])
        
        A1=projection([-b/2,a/2],-np.pi/4,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
        
        A2=projection([-b/2,-a/2],-np.pi/4,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1]) 
         
        A3=projection([b/2,-a/2],-np.pi/4,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
        
        A4=projection([b/2,a/2],-np.pi/4,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])

    
    
    #4ème étape : on fait tourner la voiture jusqu'à ce que A1, A4 et C soient alignés
    n=0
    while(((L_1-A4[0]))/norme(C,A4))-((L_1-A1[0])/(norme(C,A1)))<0:
    
        
        angle=(-np.pi/4)+((n*np.pi)/180)
    
        A1=projection([-b/2,a/2],angle,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])

        A2=projection([-b/2,-a/2],angle,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])
    
        A3=projection([b/2,-a/2],angle,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
    
        G=projection([0,0],angle,G)
        G_x.append(G[0])
        G_y.append(G[1])
        
        n+=1
    print(n)
    
    
    #5ème étape : on avance jusqu'à ce que A4 soit à la limite de la distance de sécurité verticale
    while A4[1]<L_2-h:
        G[0]=G[0]+0.01
        G[1]=G[1]+0.01
        G_x.append(G[0])
        G_y.append(G[1])
        
        A1=projection([-b/2,a/2],angle,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
    
        A2=projection([-b/2,-a/2],angle,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])
      
        A3=projection([b/2,-a/2],angle,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
        
       
    #6ème étape : on fait tourner la voiture jusqu'à ce que A1, A4 et D soient alignés
    
    n=0
    while(((A1[0]))/norme(D,A1))-((A4[0])/(norme(D,A4)))<0:
        
        angle2=angle+((n*np.pi)/180)
    
        A1=projection([-b/2,a/2],angle2,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
    
        A2=projection([-b/2,-a/2],angle2,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])
      
        A3=projection([b/2,-a/2],angle2,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle2,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
    
        G=projection([0,0],angle2,G)
        G_x.append(G[0])
        G_y.append(G[1])
        
        n-=1
    print(n)
    
    #7ème étape : on recule jusqu'à ce que A1 soit à la limite de la distance de sécurité verticale
    
    while A1[1]<L_2-h:
        G[0]=G[0]-0.01
        G[1]=G[1]+0.01
        G_x.append(G[0])
        G_y.append(G[1])
        
        A1=projection([-b/2,a/2],angle2,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
    
        A2=projection([-b/2,-a/2],angle2,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1]) 
    
        A3=projection([b/2,-a/2],angle2,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle2,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])    
       
    #8ème étape : on fait tourner la voiture jusqu'à ce que A1, A4 et C soient alignés
    n=0
    while(((L_1-A4[0]))/norme(C,A4))-((L_1-A1[0])/(norme(C,A1)))<0:
    
        
        angle3 = angle2 + ((n*np.pi)/180)
    
        A1=projection([-b/2,a/2],angle3,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])

        A2=projection([-b/2,-a/2],angle3,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])
    
        A3=projection([b/2,-a/2],angle3,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle3,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
    
        G=projection([0,0],angle3,G)
        G_x.append(G[0])
        G_y.append(G[1])
        
        n+=1
    print(n)
    
    
    #9ème étape : on avance jusqu'à ce que A4 soit à la limite de la distance de sécurité verticale
    while A4[1]<L_2-h:
        G[0]=G[0]+0.01
        G[1]=G[1]+0.01
        G_x.append(G[0])
        G_y.append(G[1])
        
        A1=projection([-b/2,a/2],angle3,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
    
        A2=projection([-b/2,-a/2],angle3,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])
      
        A3=projection([b/2,-a/2],angle3,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle3,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
        
     
    #10ème étape : on fait tourner la voiture jusqu'à ce que A1, A4 et D soient alignés
    
    n=0
    while(((A1[0]))/norme(D,A1))-((A4[0])/(norme(D,A4)))<0:
        
        angle4=angle3+((n*np.pi)/180)
    
        A1=projection([-b/2,a/2],angle4,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
    
        A2=projection([-b/2,-a/2],angle4,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1])
      
        A3=projection([b/2,-a/2],angle4,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle4,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
    
        G=projection([0,0],angle4,G)
        G_x.append(G[0])
        G_y.append(G[1])
        
        n-=1
    print(n)
    
    #11ème étape : on recule jusqu'à ce que A1 soit à la limite de la distance de sécurité verticale
    
    while A1[1]<L_2-h:
        G[0]=G[0]-0.01
        G[1]=G[1]+0.01
        G_x.append(G[0])
        G_y.append(G[1])
        
        A1=projection([-b/2,a/2],angle4,G)
        A1_x.append(A1[0])
        A1_y.append(A1[1])
    
        A2=projection([-b/2,-a/2],angle4,G)
        A2_x.append(A2[0])
        A2_y.append(A2[1]) 
    
        A3=projection([b/2,-a/2],angle4,G)
        A3_x.append(A3[0])
        A3_y.append(A3[1])
    
        A4=projection([b/2,a/2],angle4,G)
        A4_x.append(A4[0])
        A4_y.append(A4[1])
    

    
    #8ème étape : on se remet à un angle 0 et on place la voiture au milieu de la place
    
    A1=projection([-b/2,a/2],0,G)
    A1_x.append(A1[0])
    A1_y.append(A1[1])
    
    A2=projection([-b/2,-a/2],0,G)
    A2_x.append(A2[0])
    A2_y.append(A2[1])
      
    A3=projection([b/2,-a/2],0,G)
    A3_x.append(A3[0])
    A3_y.append(A3[1])
    
    A4=projection([b/2,a/2],0,G)
    A4_x.append(A4[0])
    A4_y.append(A4[1])
    
    G=projection([0,0],0,G)
    G_x.append(G[0])
    G_y.append(G[1])
    
    while G[0]<L_1/2 :
        G[0]=G[0]+0.01
        G_x.append(G[0])
        G_y.append(G[1])
        
        A1=projection([-b/2,a/2],0,G)
        A2=projection([-b/2,-a/2],0,G)
        A3=projection([b/2,-a/2],0,G)
        A4=projection([b/2,a/2],0,G)
       
    Voiture_x=[A1[0],A2[0],A3[0],A4[0],A1[0]]
    Voiture_y=[A1[1],A2[1],A3[1],A4[1],A1[1]]
    plt.plot(Voiture_x,Voiture_y, label="Voiture")
        
    return(G_x,G_y,A1_x,A1_y,A2_x,A2_y,A3_x,A3_y,A4_x,A4_y)
    
   
#tracé de la courbe de position de G

#(a,b,L_1,L_2,h,G_x0,G_y0,e)


plt.figure('Position x de G, Position y de G')
#AUTOPARK=auto_park(1.720,3.747,5,2,0.02,8,-1.36,0.05)
AUTOPARKbis=auto_park(0.132,0.288,0.385,0.154,0.0015,0.524,-0.156,0.05)
plt.plot(AUTOPARKbis[0],AUTOPARKbis[1],label="Position de G")
plt.xlabel('Position x (m)')
plt.ylabel('Position y (m)')

#tracé de la place de parking
plt.plot(Tracé_place()[0],Tracé_place()[1], label="Place de parking")


""" partie pour exporter les données pour traitemenent ultérieur sous open office

with open('H:\TIPE\Programme Python\Gy.txt','w') as file :
    for item in AUTOPARKbis[1] :
        item=str(item)
        file.write(item+"\n")
"""


plt.gca().invert_xaxis()
plt.gca().invert_yaxis()
plt.legend()
plt.grid()
plt.show()



    
        
        
        

        
        
        
        
        
        
        
        
        