import numpy as np

import matplotlib.path as mplPath

'''
Global function to call in loop control
'''
def intersec_line(a1,x1,y1,a2,x2,y2):
  #give the intersection point of two straight line
  # assuming we have the components of lines functions as:
  #(L1) : y=a1(x-x1)+y1
  #(L2) : y=a2(x-x2)+y2
  #if L1 and L2 are parallel, return (x2,y2) coordinate
  if a1==a2:
    x=x2
    y=y2
  elif a1==None:
    x=x1
    y=a2*(x-x2)+y2
  elif a2==None:
    x=x2
    y=a1*(x-x1)+y1
  else:
    x=(a1*x1-a2*x2-y1+y2)/(a1-a2)
    y=a1*(x-x1)+y1
    
  return [x,y]
      
def projection_convex(xp,yp,xi,yi,xv,yv):
  #we search the projection of the point p=(xp,yp) in direction of i=(xi,yi)
  #on the convex polygon define by the vertices (xv,yv)
  #i has to be inside the convex polygon
  #if p is iside the convex polygon, the function return p
  
  poly=zip(xv,yv)
  bbPath = mplPath.Path(np.array(poly))
#        if p.contains_points([(xi, yi)])==0:
#            error(message('i=(xi,yi) is not inside the polygon=(xv,yv)'));
  
  ii=0
  if bbPath.contains_point((xp,yp))==1:
    x0=xp
    y0=yp
  else:
    if xp==xi:
      a0=None
    else:
      a0=(yp-yi)/(xp-xi)
       
    [x0,y0,ii]=search_projection_convex(xp,yp,xi,yi,a0,xv,yv)

  return [x0,y0,ii]
        
def search_projection_convex(xp,yp,xi,yi,a,xv,yv):
  #we search the closest point to ZMP projected on the convex
  #hull edges along the perpendicular to ankle segment
  x0=[]
  y0=[]
  norm0=np.inf
  for j in range(0,len(xv)-1):
      if xv[j+1]==xv[j]:
        a0=None
      else:
        a0=(yv[j+1]-yv[j])/(xv[j+1]-xv[j])
        
      [x0_,y0_]=intersec_line(a0,xv[j],yv[j],a,xp,yp)
      
      if [x0_,y0_]<>[xp,yp]:                
        v1=[x0_-xv[j],y0_-yv[j]]
        v2=[x0_-xv[j+1],y0_-yv[j+1]]
        
        if v1[0]*v2[0]<0 or v1[1]*v2[1]<0 or v1[0]*v2[0]+v1[1]*v2[1]==0:
            if norm0>((xp-x0_)**2+(yp-y0_)**2+(xi-x0_)**2+(yi-y0_)**2):
                norm0=((xp-x0_)**2+(yp-y0_)**2+(xi-x0_)**2+(yi-y0_)**2)
                x0=x0_
                y0=y0_
                ii=j+1
            
        else:
            if ((xv[j+1]-x0_)**2+(yv[j+1]-y0_)**2)>((xv[j]-x0_)**2+(yv[j]-y0_)**2):
                x0_=xv[j];
                y0_=yv[j];
            else:
                x0_=xv[j+1];
                y0_=yv[j+1];
            
            if norm0>((xp-x0_)**2+(yp-y0_)**2+(xi-x0_)**2+(yi-y0_)**2):
                norm0=((xp-x0_)**2+(yp-y0_)**2+(xi-x0_)**2+(yi-y0_)**2)
                x0=x0_
                y0=y0_
                ii=j+1
                
  return [x0,y0,ii]