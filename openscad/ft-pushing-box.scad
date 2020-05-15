/** ft-pushing-box.scad
 * 
 * A sliding-force measuring box for our robot pushing experiments.
 *
 * 2020.02.15 - new (copied from adjustable-ft-v1)
 *
 * (c) 2019,2020 fnh, hendrich@informatik.uni-hamburg.de
 */
 
 // see end of file for our #include's...

eps = 0.01;
fn  = 250;

fdm_fudge = 0.2; // extra size for bores/holes/nuts due to shrinkage
explode_distance = 0; // 0 10 20 50;
pcb_thickness = 1.0;

// disable before exporting to STL
// 
make_test_box       = false;
make_assembled_pushing_box = true;

// enable individual parts to render&export, or to redesign:
//
make_bottom_plate   = false;
make_sensor_carrier = false;
make_coins_holder   = false;
make_spring_block   = false;
make_fin_carrier    = false;
show_sensor_module  = true;



x_size = 80;
y_size = 80;
outer_wall_thickness = 2;
outer_wall_height = 13;
sensor_cutouts = [[]];
//round_cutouts = [[55,0,28,32], [-55,0,28,32]]; // x,y,d1,d2
round_cutouts = [[5,0,28,32]];
bulges = [[37-50,0,26,50,3]];
bores=[[28-50,18], [28-50,-18], [41.5-50,14.8], [41.5-50,-14.8]];


if (make_bottom_plate) { 
  bottom_plate( x_size = x_size, y_size = y_size, 
                z_thickness = 2,
                outer_wall_height = outer_wall_height,
                outer_wall_thickness = outer_wall_thickness,
                round_cutouts=round_cutouts,
                bulges=bulges,
                bores=bores
              );
}  
  




if (make_coins_holder)   t(-0,0,-5) color( "lightgreen" ) coins_holder();
if (make_sensor_carrier) t(-18,0,0) sensor_carrier();
if (make_spring_block)   color( "lightblue" ) spring_block();
if (make_fin_carrier)    t(-40,0,0) fin_carrier();
  


if (make_test_box) {
translate( [0,250,0] ) {
      t(0,0,0) color( "lightgray" ) 
        bottom_plate( x_size = 80, y_size = 80,
                      outer_wall_height = 3,
                      bulges=[[37-50,0,26,50,3]],
                      bores=[[28-50,18], [28-50,-18], [41.5-50,14.8], [41.5-50,-14.8]], 
                      round_cutouts = [[5,0,28,32]]
                    );   
      t(  5,0,0) color( "darkgray" )                      coins_holder();
      t(  5,0,5) rotate( [0,0,180] )   color( "skyblue" ) spring_block();
      t( 36-50,0,5) rotate( [0,0,180] )                      sensor_carrier();
      t( 49.5-50,0,5) rotate( [0,0,180] ) color( "black" )   fin_carrier();

}  
}




if (make_assembled_pushing_box) {
translate( [0,130,0] ) {
  intersection() {
    
    union() {
      t(0,0,0) color( "lightgray" ) 
        bottom_plate( outer_wall_height = 0,
                      y_size = 60,
                      bulges=[[37,0,26,50,3],[-37,0,26,50,3],
                              [20,0,2,50,4], [-20,0,2,50,4]],
                      bores=[[-28,18], [-28,-18], [-41.5,14.8], [-41.5,-14.8],
                             [ 28,18], [ 28,-18], [ 41.5,14.8], [ 41.5,-14.8]
                            ] );   
      
      t(-55,0,0) color( "darkgrey" )   coins_holder();
      t(-55,0,5) color( "skyblue" )    spring_block();
      t(-36,0,5)                       sensor_carrier();
      t(-49.5,0,5) color( "blue" )    fin_carrier();
      
      t( 55,0,0) color( "darkgrey" )                      coins_holder();
      t( 55,0,5) rotate( [0,0,180] )   color( "skyblue" ) spring_block();
      t( 36,0,5) rotate( [0,0,180] )                      sensor_carrier();
      t( 49.5,0,5) rotate( [0,0,180] )   fin_carrier();
    } // union: all of ours
  
    // intersection cube to show inside of ours...
    t(0,0,0) cube( size=[200,200,225], center=true ); // 200, 100/50, 100/25
  } // intersection
} // translate
} // show_assembled_pushing_box
  

// if (show_sensor_module) 2DOF_sensor_module();


module t( dx, dy, dz ) {
  translate( [dx,dy,dz] ) children(); 
}



/* ************************************************************** */
/* ************************************************************** */
/* ************ SENSOR CARRIER ********************************** */
/* ************************************************************** */
/* ************************************************************** */


module sensor_carrier( 
  scx = 10.8,
  scy = 20,
  sct =  1,  // core housing plate thickness

  ocy = 40,  // mounting plate width
  oct =  2,  // mounting plate thickness
  bore = 5,
  col = "orange" // [1.0,0.7,0.3, 1] 
)
{
  bx = 11.9; by = 6.3; bz = 10.8; // Vishay TCST 1103 outer dimensions
  scz = 2*sct + bx;
  fdm_gap = 0.2;
  
  difference() {
    color( col )
    union() {
      // main sensor housing block
      translate( [-scx/2,0,scz/2] )
        cube( size=[scx, scy, scz], center=true );
      
      // outer mounting plate (with screw bores)
      translate( [-scx/2,0,oct/2] )
        cube( size=[scx, ocy, oct], center=true );
      
    }

    // two cutouts for the sensors    
    translate( [-scx/2, 1.5*2.54,scz/2] )
      cube( size=[scx+eps, by+fdm_gap, bx+fdm_gap], center=true );
    translate( [-scx/2,-1.5*2.54,scz/2] )
      cube( size=[scx+eps, by+fdm_gap, bx+fdm_gap], center=true );
    
    // extra corner cutouts at the corners (avoid fdm shrinkage)
    ddd = 0.5; // ok
    // ddd = 0.8; // too thin
    for( z=[-bx/2, bx/2] ) {
      for( x=[ -1.5*2.54-by/2, -1.5*2.54+by/2, 1.5*2.54-by/2, 1.5*2.54+by/2] ) {
        translate( [-eps, x, z+scz/2] ) rotate( [0,-90,0] ) 
          cylinder( d=ddd, h=scx+2*eps, center=false, $fn=20 );
      }
    }    

    // mounting plate screw holes (extra large for adjustment)
    for( y=[-ocy/2+scx/2, ocy/2-scx/2] )
      translate( [-scx/2, y, -eps] )
        cylinder( d=bore, h=oct+2*eps, center=false, $fn=20 );
    
    // mounting plate m2.5 hex nut cavities

    
    // central fin cutout
    xx = 8;
    translate( [-scx+xx/2,0,scz/2] )
      cube( size=[xx+eps, scy+eps, 3], center=true );
  }
  
  if (show_sensor_module) {
    translate( [pcb_thickness+eps,-3*2.54,-1.5*2.54+11.9/2+sct+2*explode_distance] ) rotate( [0,-90,0] ) 
      2DOF_sensor_module();
  }
} // sensor_carrier



/* ************************************************************** */
/* ************************************************************** */
/* ************ COINS_HOLDER ************************************ */
/* ************************************************************** */
/* ************************************************************** */


/**
 * cylindrical coin-holder for n_coins coins of given size and weight.
 * Slanted outer walls for easier sliding (d2,d3,t).
 * Centered on x,y and on top of z=0.
 */
module coins_holder(
  d3     = 28,
  d2     = 24,
  d1     = 21.75,
  t      = 1.0,   // base plate thickness
  h1     = 4.0,   // coin-cavity height

  connector_blocks = [[0,3,3,2], [90,3,3,2], [270,3,3,2]],

  d_coin = 21.25, // 5 euro cent
  h_coin =  1.67, // 
  weight_coin = 3.92, // grams
  n_coins = 2
)
{
  // main coin-holder (bottom part)
  difference() {
    cylinder( d1=d2, d2=d3, h=t+h1, center=false, $fn=100 ); 
    translate( [0,0,t] )
      cylinder( d=d1, h=h1+eps, center=false, $fn=100 );
  }
  if (h1 < (n_coins*h_coin)) {
    echo( "WARNING: coins_holder h1 too small for ", n_coins, " coins: ", h1 ); 
  }
  // connector-blocks: angle, sx,sy,sz
  for( i=[0:len(connector_blocks)-1] ) {
    cb = connector_blocks[i];
    phi = cb[0]; 
    sx = cb[1];
    sy = cb[2];
    sz = cb[3];
    rotate( [0,0,phi] ) translate( [d1/2+sx/2, 0, t+h1+sz/2] )
      cube( size=[sx,sy,sz+eps], center=true );
  }
} // coins_holder


/* ************************************************************** */
/* ************************************************************** */
/* ************ SPRING_BLOCK ************************************ */
/* ************************************************************** */
/* ************************************************************** */


module spring_block(
  h   = 10,  // total lever height
  lxx = 29.4, // length of x levers  (was 23)
  lxw =  0.7, // width  of x levers
  lxh =  2, // height of x levers
  nxz =  2, // z number of x levers (1 or 2)

  lyy = 21.7, // length of y levers
  lyw =  0.7, // width  of y levers
  lyh =  2, 
  nyx =  1, // x number of parallel y levers,  
  nyz =  2, // z number of y levers (1 or 2)

  sxx = 10, // size of central plate
  syy = 27,
  szz =  2,
  s_cutouts = [[21.5, 90,3,3,2], [21.5, 270,3,3,2]], // [r,phi,sx,sy,sz]
  ccx =  1.5, // inner/moving/outer column width
  ccy =  1.5,
  scw =  9, // screw column width
  scb = 2.7, // screw column bore
  scn = M25NUTe + fdm_fudge,
)
{
  difference() {
    union() {
      // central plate
      translate( [0,0,szz/2] )
        cube( size=[sxx, syy, szz], center=true );

      // central column  
      translate( [0,0,h/2] )
        cube( size=[max(ccx,sxx), ccy, h], center=true );
  
      // fin carrying wall
      fcw = 1;
      translate( [sxx/2-fcw/2,0,h/2] )
        cube( size=[fcw, syy, h], center=true );
    }
    
    if (len(s_cutouts) > 0) {
      for( m=[0:len(s_cutouts)-1] ) {
        cutout = s_cutouts[m];
        r   = cutout[0];
        phi = cutout[1];
        sx  = cutout[2];
        sy  = cutout[3];
        sz  = cutout[4];
        rotate( [0,0,phi] ) translate( [r/2+sx/2, 0, szz/2] )
          cube( size=[sx+eps,sy+eps,szz+5*eps], center=true );
      } 
    }
  }

  // y-levers: need to be printed as bridges...
  for( i = [-(nyx-1)/2 : (nyx-1)/2] ){
    dx = i * (ccx - lyw) / max(1, nyx-1);

    for( j = [0 : nyz-1] ) {
      // topmost lever: (h-lyh/2) bottommost: (szz+1+lyh/2)
      // ddz = (topmost - bottommost) = h - szz + 1
      dz = (h-lyh/2) - j*(h-szz-1-lyh);
      
      translate( [dx, lyy/2+ccy/2,dz] ) 
        cube( size=[lyw,lyy+eps,lyh], center=true );
      translate( [dx,-lyy/2-ccy/2,dz] ) 
        cube( size=[lyw,lyy+eps,lyh], center=true );
    }
  }
  
  // outer moving columns
  for( dy = [-lyy-ccy, lyy+ccy] ) {
    translate( [ccx/2-lyw/2,dy,h/2] ) 
      cube( size=[ccx, ccy, h], center=true );
  }
  
  // x-levers: need to be printed as bridges... centered in z
//  for( dy = [-lyy-1.5*ccy+lxw/2, lyy+1.5*ccy-lxw/2] ) {
//    hh = h - lxh/2;
//    // hh = szz + 1 + lxh/2;
//    translate( [ccx/2 + lxx/2, dy, hh] ) 
//      cube( size=[lxx+eps, lxw, lxh], center=true );
//  }
  
  for( dy = [-lyy-1.5*ccy+lxw/2, lyy+1.5*ccy-lxw/2] ) {
    for( j = [0 : nxz-1] ) {
      // topmost lever: (h-lyh/2) bottommost: (szz+1+lyh/2)
      // ddz = (topmost - bottommost) = h - szz + 1
      dz = (h-lxh/2) - j*(h-szz-1-lxh);
      translate( [ccx/2 + lxx/2, dy, dz] ) 
        cube( size=[lxx+eps, lxw, lxh], center=true );
    }
  }

  
  
  
  scx = ccx/2 + lxx + ccx - scw/2; // x-position of screw columns
  scy = lyy + ccy/2 - scw/2; // y-position of screw columns
  
  difference() {
    union() {
      // fixed outer columns
      for( dy = [-lyy-ccy, lyy+ccy] ) {
        translate( [lxx+ccx,dy,h/2] ) 
          cube( size=[ccx, ccy, h], center=true );
      }
      // mounting/connecting plate
      cpx = max( ccx, scw ); 
      cpy = 2*lyy + ccy;
      translate( [lxx+ccx+ccx/2 - cpx/2,0,szz/2] )
        cube( size=[cpx, cpy+eps, szz], center=true );
      
      // screw columns
      for( dy = [-scy, +scy] ) {
        translate( [scx,dy,h/2] ) 
          cube( size=[scw, scw, h], center=true );
      }
    }
    
    // screw bores and nut cavities
    for( dy = [-scy, +scy] ) {
      translate( [scx,dy,h/2] ) 
        cylinder( d=scb, h=h+eps, center=true, $fn=20 );
      translate( [scx,dy,h/2] ) 
        cylinder( d=scn, h=h+eps, center=false, $fn=6 );
    }
  } // difference

} // spring_block


/* ************************************************************** */
/* ************************************************************** */
/* ************ FIN CARRIER ************************************* */
/* ************************************************************** */
/* ************************************************************** */


module fin_carrier(
  sx = 1,
  sy = 27,
  sz = 10,
  fz = 1 + 11.9/2,
  fx1 = 5,
  fy1 = 8,
  fx2 = 8,
  fy2 = 4,
  fin_thickness = 1.0,
  show_rgb_axis_colors = true,
)
{
  // carrier plate, glued to spring-block central-plate
  translate( [0,0,sz/2] )
    cube( size=[sx, sy, sz], center = true );

  // short fin (measures x-axis motion)
  color( "red" )
  translate( [sx/2+fx1/2, fy1/2, fz] ) 
    cube( size=[fx1,fy1,fin_thickness], center=true );
  
  // long fin (measures y-axis motion)
  color( "green" )
  translate( [sx/2+fx2/2, -fy2/2, fz] ) 
    cube( size=[fx2,fy2,fin_thickness], center=true );
  
}


/* ************************************************************** */
/* ************************************************************** */
/* ************ SPRING BLOCK V1********************************** */
/* ************************************************************** */
/* ************************************************************** */




module spring_block_v1(
  h   = 10,  // total lever height
  lxx = 23, // length of x levers
  lxw =  0.8, // width  of x levers
  lxh =  6, // height of x levers
  lyy = 20, // length of y levers
  lyw =  0.8, // width  of y levers
  lyh =  1.5, 
  nyx =  2, // x number of parallel y levers,  
  nyz =  2, // z number of y levers (1 or 2)

  sxx = 10, // size of central plate
  syy = 27,
  szz =  2,
  s_cutouts = [[21.5, 90,3,3,2], [21.5, 270,3,3,2]], // [r,phi,sx,sy,sz]
  ccx =  5, // inner/moving/outer column width
  ccy =  3,
  scw =  7, // screw column width
  scb = 2.7, // screw column bore
  scn = M25NUTe + fdm_fudge,
)
{
  difference() {
    union() {
      // central plate
      translate( [0,0,szz/2] )
        cube( size=[sxx, syy, szz], center=true );

      // central column  
      translate( [0,0,h/2] )
        cube( size=[max(ccx,sxx), ccy, h], center=true );
  
      // fin carrying wall
      fcw = 1;
      translate( [sxx/2-fcw/2,0,h/2] )
        cube( size=[fcw, syy, h], center=true );
    }
    
    if (len(s_cutouts) > 0) {
      for( m=[0:len(s_cutouts)-1] ) {
        cutout = s_cutouts[m];
        r   = cutout[0];
        phi = cutout[1];
        sx  = cutout[2];
        sy  = cutout[3];
        sz  = cutout[4];
        rotate( [0,0,phi] ) translate( [r/2+sx/2, 0, szz/2] )
          cube( size=[sx+eps,sy+eps,szz+5*eps], center=true );
      } 
    }
  }

  // y-levers: need to be printed as bridges...
  for( i = [-(nyx-1)/2 : (nyx-1)/2] ){
    dx = i * (ccx - lyw) / max(1, nyx-1);

    for( j = [0 : nyz-1] ) {
      // topmost lever: (h-lyh/2) bottommost: (szz+1+lyh/2)
      // ddz = (topmost - bottommost) = h - szz + 1
      dz = (h-lyh/2) - j*(h-szz-1-lyh);
      
      translate( [dx, lyy/2+ccy/2,dz] ) 
        cube( size=[lyw,lyy,lyh], center=true );
      translate( [dx,-lyy/2-ccy/2,dz] ) 
        cube( size=[lyw,lyy,lyh], center=true );
    }
  }
  
  // outer moving columns
  for( dy = [-lyy-ccy, lyy+ccy] ) {
    translate( [0,dy,h/2] ) 
      cube( size=[ccx, ccy, h], center=true );
  }
  
  // x-levers: need to be printed as bridges... centered in z
  for( dy = [-lyy-ccy, lyy+ccy] ) {
    translate( [ccx/2 + lxx/2, dy, h/2] ) // h-lxh/2 ?
      cube( size=[lxx, lxw, lxh], center=true );
  }
  
  scx = ccx/2 + lxx + ccx - scw/2; // x-position of screw columns
  scy = lyy + ccy/2 - scw/2; // y-position of screw columns
  
  difference() {
    union() {
      // fixed outer columns
      for( dy = [-lyy-ccy, lyy+ccy] ) {
        translate( [lxx+ccx,dy,h/2] ) 
          cube( size=[ccx, ccy, h], center=true );
      }
      // mounting/connecting plate
      cpx = max( ccx, scw ); 
      cpy = 2*lyy + ccy;
      translate( [lxx+ccx+ccx/2 - cpx/2,0,szz/2] )
        cube( size=[cpx, cpy+eps, szz], center=true );
      
      // screw columns
      for( dy = [-scy, +scy] ) {
        translate( [scx,dy,h/2] ) 
          cube( size=[scw, scw, h], center=true );
      }
    }
    
    // screw bores and nut cavities
    for( dy = [-scy, +scy] ) {
      translate( [scx,dy,h/2] ) 
        cylinder( d=scb, h=h+eps, center=true, $fn=20 );
      translate( [scx,dy,h/2] ) 
        cylinder( d=scn, h=h+eps, center=false, $fn=6 );
    }
  } // difference

} // spring_block_v1





/* ************************************************************** */
/* ************************************************************** */
/* ************ BOTTOM PLATE ************************************ */
/* ************************************************************** */
/* ************************************************************** */


/**
 * the bottom plate of the box, centered in x and y and above z=0.
 * Given size and optional outer walls.
 * Sensor cutouts at the given locations and sizes.
 */
module bottom_plate(
  x_size = 150,
  y_size = 120,
  z_thickness = 2,
  outer_wall_thickness = 2,
  outer_wall_height = 30,
  sensor_cutouts = [], // [[0,0,20,50], [60,60,12,12]], // [x,y,dx,dy]*
  round_cutouts = [[55,0,28,32], [-55,0,28,32]], // x,y,d1,d2
  bulges = [], // [x,y,dx,dy,dz]
  bores = [],
  dummy = 0
)
{
  
  difference() {
    union() {
      // main bottom plate
      translate( [0,0,z_thickness/2] )
        cube( size=[x_size, y_size, z_thickness], center=true );
      
      if (len(bulges) > 0) {
        for( m=[0:len(bulges)-1] ) {
          b = bulges[m]; // [x,y,sx,sy,sz]
          x = b[0];
          y = b[1];
          sx = b[2];
          sy = b[3];
          sz = b[4];
          translate( [x,y,z_thickness+sz/2] )
            cube( size=[sx,sy,sz+eps], center=true );
        } 
      }
    }
    
    // optional rectangular sensor cutouts
    if (len(sensor_cutouts) > 0) {
      for( m=[0:len(sensor_cutouts)-1] ) {
        cutout = sensor_cutouts[m];
        translate( [cutout[0],cutout[1],z_thickness/2] )
          cube( size=[cutout[2],cutout[3],z_thickness+eps], center=true );
      }
    }
    
    // optional round sensor cutouts
    if (len(round_cutouts) > 0) {
      for( m=[0:len(round_cutouts)-1] ) {
        cutout = round_cutouts[m];
        d1 = cutout[2];
        d2 = len(cutout) >= 3 ? cutout[3] : cutout[2]; // conical if requested
        translate( [cutout[0],cutout[1],z_thickness/2] )
          cylinder( d=d1,d2=d2,h = z_thickness+eps, center=true, $fn=100 );
        translate( [cutout[0],cutout[1],z_thickness] ) // extra height for bulges, if any
          cylinder( d=d2, h=20, center=false, $fn=100 );
      }
    }
    
    // optional screw bores, currently hardcoded to M2.5 screws
    if (len(bores) > 0) {
      for( m=[0:len(bores)-1] ) {
         bore = bores[m];
         x    = bore[0];
         y    = bore[1];
         db   = M25BORE + fdm_fudge;
         hd   = M25SCREWdk + fdm_fudge;
         hh   = M25SCREWk + 0.2;

         // screw bore through
         translate( [x,y,-eps] )
           cylinder( d=db, h=10, center=false, $fn=20 );
        
         translate( [x,y,hh] )
           cylinder( d=db-0.5, h=10, center=false, $fn=20 );
        
         // screw head cavity
         translate( [x,y,-1.1*eps] )
           cylinder( d=hd, h=hh, center=false, $fn=20 );
      } 
    }
    
    
  } // difference

  // outer walls
  wt = outer_wall_thickness/2;
  for( x=[-x_size/2+wt, x_size/2-wt] ) {
    translate( [x,0,outer_wall_height/2] )
      cube( size=[outer_wall_thickness, y_size, outer_wall_height], center=true );
  }
  for( y=[-y_size/2+wt, y_size/2-wt] ) {
    translate( [0,y,outer_wall_height/2] )
      cube( size=[x_size, outer_wall_thickness, outer_wall_height], center=true );
  }
  
}


/* ************************************************************** */
/* ************************************************************** */
/* ************ OTHER STUFF ************************************* */
/* ************************************************************** */
/* ************************************************************** */


// main dimensions
//

M2NUTs = 4;    // diameter across sides
M2NUTe = 4.38; // outer diameter across edges
M2NUTm = 1.6;  // nut height
M2BORE = 2.0;
M2SCREWdk = 3.8; // inbus screw head diameter
M2SCREWk  = 2.0; // inbus screw head height

M25NUTs = 5;
M25NUTe = 5.45;
M25NUTm = 2;
M25BORE = 2.5;
M25SCREWdk = 4.5; // inbus screw head diameter
M25SCREWk  = 2.5; // inbus screw head height

M3NUTs = 5.5;
M3NUTe = 6.08;
M3NUTm = 2.4;
M3BORE = 3.0;   // add fudge / redefine on command line as neede
M3SCREWdk = 5.5; // inbus screw head diameter
M3SCREWk  = 3.0; // inbus screw head height

M4NUTs = 7.0;
M4NUTe = 7.8;
M4NUTm = 3.2;
M4BORE = 4.0;
M4SCREWdk = 7.0; // inbus screw head diameter
M4SCREWk  = 4.0; // inbus screw head height

M5NUTs = 8.0;
M5NUTe = 8.63;
M5NUTm = 4.0;
M5BORE = 5.0;
M5SCREWdk = 8.5; // inbus screw head diameter
M5SCREWk  = 5.0; // inbus screw head height




/**
 * a small adjustable (!) carrier for two Vishay TCST1103
 * optocouplers mounted on a breaboard PCB. The sensor slots
 * are aligned along the y-axis.
 * Sensor dimensions are bx 11.0, by 6.3, bz 3.1 mm.
 *
 * To ensure printability on FDM printers, we use slight
 * "corner cutouts". This should help against material shrinkage
 * during cooling of the printed part, and will allow us to
 * "file" the carrier a bit for a snug fit of the sensors.
 * The middle wall can be broken away if needed.
 */
module 2DOF_sensor_carrier( 
  xx = 15, yy = 17, hh =10,  // main carrier block (v2 had hh=10)
  show_sensor=true 
)
{
  bx = 11.9;
  by = 6.3;
  bz = 3.1;
  // baseplate with cutouts for the sensors
  //
  
  dd = 0.6; // corner cutout cylinder diameter
color( [1.0,0.4,0.1] ) {
  difference() {
    2DOF_sensor_carrier_box();
    
    // central slot cutout, slot width 3.1, we use 6.2 here
    translate( [0,0,(hh)/2 ] )
      cube( size=[6.2, 7*2.54 + 6.3, hh+2*eps], center=true );
    
    // subtract the sensor module itself
    translate( [-1.5*2.54,-3*2.54,-pcb_thickness-eps] )  
      2DOF_sensor_module( show_optical_axis=false );
    
    // corner cutouts
    for( x=[-bx/2, bx/2] ) {
      for( y=[ -1.5*2.54-by/2, -1.5*2.54+by/2, 1.5*2.54-by/2, 1.5*2.54+by/2] ) {
        translate( [x,y,-eps] ) 
          cylinder( d=dd, h=hh+2*eps, center=false, $fn=20 );
      }
    }    
  }
  
}
  
  if (show_sensor) {
    translate( [-1.5*2.54,-3*2.54,-pcb_thickness-explode_distance] )  2DOF_sensor_module();
  }
}




module 2DOF_sensor_carrier_mounting_block(
  xx = 15, yy = 16, hh = 10,  // main carrier block
  xx2 = 8.0, yy2 = 8.0,       // side mounting screw blocks
  extra_y = 1.0, // make outer walls a bit thicker
)
{
  dyy = yy/2 + yy2/2;

  difference() {
    translate( [-xx/2-xx2/2, 0, hh/2] ) 
      cube( size=[xx2, yy+2*yy2-2+2*extra_y, hh-2], center=true );

    // hex nut cutout 
    m3nut = M25NUTe + fdm_fudge;
    for( dy = [-dyy, +dyy] ) {
      translate( [-xx/2+1, dy, hh/2] ) 
          rotate( [0,270,0] ) 
            cylinder( d=m3nut, h=xx2+2+5*eps, center=false, $fn=6 );
    }
  }
  
  
  for( dy = [-dyy, +dyy] ) {
    translate( [-xx/4, dy, hh/2] ) 
    difference() {
      // main body
      translate( [0,extra_y/2*sign(dy),0] )
      cube( size=[xx/2, yy2-2+extra_y, hh-2], center=true );
    
      // screw bore through the block
      m3bore = 3.0 + fdm_fudge;
      translate( [0, 0, 0] )
        rotate( [0,90,0] ) 
          cylinder( d=m3bore, h=xx/2+eps, center=true, $fn=20 );

      // hex nut cutout 
      // translate( [-xx/4, 0, 0] )
      //   rotate( [0,90,0] ) 
      //     cylinder( d=5.5, h=5.0, center=true, $fn=6 );
    } // difference
  } // for
  
}
 
   





/** 
 * a small (4x8 holes) breadboard to carry two Vishay TCST1103 
 * optocouplers. The slots are aligned along the y-axis.
 * When mechanically inserted into the sensor base plate, 
 * this serves to measure y- and z-deflection of one elastic arm.
 * Note: The origin of this assembly is at the bottom-left hole (bore)
 * of the breadboard. 
 * Cables are expected to be soldered from the bottom, or into
 * the "extra" four holes at ny=7.
 */
module 2DOF_sensor_module( show_optical_axis=true ) {
    dx1 =  1.5 * 2.54; // x and y offsets of the optocouplers
    dy1 =  1.5 * 2.54;    
    dy2 =  4.5 * 2.54;    

    translate( [0,1*2.54,0] ) breadboard( nx=4, ny=5, z=pcb_thickness, bore=1.0, fn=15, epoxy=false );
    translate( [dx1, dy1, pcb_thickness] ) rotate( [0,0,180] ) vishay_TCST1103( show_optical_axis=show_optical_axis, wire_length=2 );
    translate( [dx1, dy2, pcb_thickness] ) rotate( [0,0,0] ) vishay_TCST1103( show_optical_axis=show_optical_axis, wire_length=2 );
}    
    
    





module ring( d_outer, d_inner, h, fn=fn, center ) {
  difference() {
    cylinder( d=d_outer, h=h, $fn=fn, center=center );
    translate( [0,0,-eps] ) cylinder( d=d_inner, h=h+2*eps, $fn=fn, center=center );

  }
}



module gray04() {
  if (use_colors) color( [0.3,0.3,0.3] ) children();
  else children();
}

module gray05() {
  if (use_colors) color( [0.5,0.5,0.5] ) children();
  else children();
}

module gray06() {
  if (use_colors) color( [0.7,0.7,0.7] ) children();
  else children();
}

module gray08() {
  if (use_colors) color( [0.8,0.8,0.8] ) children();
  else children();
}

module red06() {
  if (use_colors) color( [0.8,0.3,0.3] ) children();
  else children();
}
    
    
include <breadboard.scad>
include <optokoppler.scad>
include <electronics.scad>
//include <ati-nano17e.scad>
