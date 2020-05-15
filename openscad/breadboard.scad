/* breadboard.scad - 3D models of different prototyping breadboards.
 *
 * 2017.12.24 - refactored into standalone file/module, better params
 * 2017.10.16 - created
 *
 * Note: depending on size, this module generates _a lot_
 * of cylindrical holes. Rendering performance can be an
 * issue, even with fn=10 or so...
 * 
 * (c) 2017, fnh, hendrich@informatik.uni-hamburg.de
 */


// global openscad parameters
eps = 0.01;


// breadboard_demo();
// eurokarte_triple();

module eurokarte_triple() {
  xsize = 160.0;
  ysize = 100.0;
  nx = floor( xsize / 2.54 );
  ny = floor( ysize / 2.54 );
  echo( nx ); echo ( ny );
  xpadding = (xsize - nx*2.54) / 2;
  ypadding = (ysize - ny*2.54) / 2;
  echo( xpadding ); echo( ypadding );

  breadboard( nx=nx, ny=ny, z=0.8, delta=2.54, bore=1.2, fn=12, 
              epoxy=false, triple=true, 
              xpadding=xpadding, ypadding=ypadding );
}


module breadboard_demo() {
translate( [ 0,0,0] ) breadboard( nx=4, ny=9, z=0.8, fn=9, epoxy=false, triple=true );
translate( [15,0,0] ) breadboard( nx=4, ny=3, z=0.8, delta=2.54, single=true, bore=1.0, xpadding=0.0, ypadding=0.0, fn=10 );
translate( [30,0,0] ) breadboard( nx=4, ny=5, z=0.8, delta=2.54, epoxy=false, bore=1.3, xpadding=0.0, ypadding=0.0, fn=10 );
translate( [15,10,0]) breadboard( nx=4, ny=5, z=0.8, delta=1.27, triple=true, bore=0.7, xpadding=0.0, ypadding=0.0, fn=10 );
}



/**
 * Simplified prototyping breadboard. 
 * Origin is at hole(0,0) (=bottom left hole) with the board 
 * extending to (nx-1, ny-1, z), plus any x/y padding.
 */
module breadboard( 
  nx       = 5,     // number of holes in x direction
  ny       = 5,     // number of holes in y direction,
  delta    = 2.54,  // hole spacing
  bore     = 1.3,   // hole diameter
  fn       = 5,     // $fn for hole rendering (performance critical!)
  z        = 0.8,   // board thickness
  xpadding = 0.1,   // extra board space at left/right ends
  ypadding = 0.1,   // extra board space at upper/lower ends,
  epoxy    = true,  // epoxy color (else hartpappe aka paper)
  one_side = true,  // copper only on one side
  triple   = false, // triple-copper-stripes 
  single   = false, // single-copper-dots
  copper_thickness = 0.05, 
  center   = false, 
)
{
  xmin = -delta/2 - xpadding;
  xmax =  (nx-1)*delta + delta/2 + xpadding;
  ymin = -delta/2 - ypadding;
  ymax =  (ny-1)*delta + delta/2 + ypadding;
  // echo( xmin ); echo( xmax ); echo( ymin ); echo( ymax ); echo( delta );
  xx = xmax - xmin;
  yy = ymax - ymin;
  // echo( xx ); echo( yy ); echo( z );

  ddx = (center == true) ? -0.5*(xmax+xmin) : 0;
  ddy = (center == true) ? -0.5*(ymax+ymin) : 0;

  translate( [ddx, ddy, 0] ) 
  difference() {  
    union() {
      // main board
      if (epoxy==true) epoxy() translate( [xmin,ymin,0] ) cube( size=[xx,yy,z], center=false );
      else             hartpappe() translate( [xmin,ymin,0] ) cube( size=[xx,yy,z], center=false );

      // copper holes / stripes (if requested)
      if (single) {
        dxy = (delta - bore) / 3;
        for( ix=[0:1:nx-1] ) {
          for( iy=[0:1:ny-1] ) {
            // echo( ix ); echo( iy );
            copper() 
              translate( [ix*delta-bore/2-dxy,iy*delta-bore/2-dxy,z-eps] ) 
                cube( size=[bore+2*dxy,bore+2*dxy,copper_thickness], center=false );
          }
        }
      }
      else if (triple) { // 3-hole stipes along y direction
        dxy = (delta - bore) / 3;
        for( ix=[0:1:nx-1] ) {
          nny = floor( ny/3 ) * 3;
          for( iy=[0:3:nny-1] ) {
            // echo( ix ); echo( iy );
            copper() 
              translate( [ix*delta-bore/2-dxy,iy*delta-bore/2-dxy,z-eps] ) 
                cube( size=[bore+2*dxy,2*delta+bore+2*dxy,copper_thickness], center=false );
          }
        }
      } // if triple
    } // union

    // the through-holes, diameter "bore", spaced at "delta"
    for( ix=[0:1:nx-1] ) {
      for( iy=[0:1:ny-1] ) {
        // echo( ix ); echo( iy );
        translate( [ix*delta,iy*delta,-eps] ) 
          cylinder( d=bore, h=z+0.1+2*eps, $fn=fn, center=false );
      }
    }
  }

} // end breadboard


module black() {
  color( [0, 0, 0] ) children();
}

module silver() {
  color( [0.9, 0.9, 0.9] ) children();
}

module copper() {
  color( [0.95, 0.7, 0.3] ) children();
}

module epoxy() {
  color( [0.2, 0.7, 0.6] ) children(); 
}

module hartpappe() {
   color( [0.7, 0.5, 0.1] ) children(); 
}

module gray07() {
  color( [0.7, 0.7, 0.7] ) children();
}

module gray02() {
  color( [0.2, 0.2, 0.2] ) children();
}


