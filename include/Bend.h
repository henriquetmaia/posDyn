// -----------------------------------------------------------------------------
// libDDG -- Bend.h
// -----------------------------------------------------------------------------
//
// Bend Constraint to achieve correct cloth dynamics between adjacent triangles
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_BEND_H
#define DDG_BEND_H

#include "Face.h"
#include "HalfEdge.h"
#include "Vector.h"
#include "Mesh.h"
#include "Constraint.h"
#include "Distance.h"
#include <math.h>
#include <iostream>

namespace DDG
{

   class Bend : public Constraint
   {
      public:

         Bend( Mesh* surface_ptr, double density, double stiff )
         {
            m_equality = true;
            m_cardinality = 6;
            m_cloth = surface_ptr;
            m_density = density;
            m_stiffness = stiff;
            m_cloth->addConstraint( new Distance( m_cloth, stiff ) ); // TODO:

            // compute initial dihedral angles
            for( EdgeIter e = m_cloth->edges.begin(); e != m_cloth->edges.end(); ++e ){
               if( !e->he->onBoundary ){

                  if( e->he->flip->face->normal().norm() != e->he->flip->face->normal().norm()  || 
                      e->he->face->normal().norm() != e->he->face->normal().norm() ){
                     e->dihedralAngle = 0.;
                     continue;
                  }
                  else{
                  e->dihedralAngle = acos( dot( e->he->face->normal() , e->he->flip->face->normal() ) );

                  // std::cout << " f1n: " << e->he->face->normal() << " f2n: " << e->he->flip->face->normal()  << " norm " << e->he->flip->face->normal().norm() << std::endl;
                  // std::cout << "input dihedralAngle: " << e->dihedralAngle << std::endl;

                  }                  

               }
               else{
                  e->dihedralAngle = 0.;
               }
            }

            for( VertexIter v = m_cloth->vertices.begin(); v != m_cloth->vertices.end(); ++v ){
               double dualArea = v->dualArea();
               v->cMass = 1.0 / ( dualArea * m_density );
               // std::cout << "cMass: " << v->cMass << std::endl;
            }

         };

         virtual ~Bend( void ){};

         virtual Vector forceFunction( Vector& position ){
            std::cerr << "forceFunction ( Vector& position ) not implemented for Constraint:: BEND " << std::endl;
            return Vector();
         };

         virtual void forceFunction( void ){

            
            // for( VertexIter v = m_cloth->vertices.begin(); v != m_cloth->vertices.end(); ++v ){
            //    std::cout << "p0: " << v->estimate << std::endl;   
            // }

            for( EdgeIter e = m_cloth->edges.begin(); e != m_cloth->edges.end(); ++e ){
               if( !e->he->onBoundary ){

                  Vector p2 = e->he->next->vertex->estimate;
                  Vector p3 = e->he->next->next->vertex->estimate;
                  Vector p4 = e->he->flip->next->next->vertex->estimate;

                  Vector n1 = cross( p2, p3 ); n1.normalize();
                  Vector n2 = cross( p2, p4 ); n2.normalize();

                  double d = dot( n1 , n2 );
                  // double constraint_func = acos( d ) - e->dihedralAngle * m_stiffness;

                  Vector q1, q2, q3, q4;

                  q3 = ( cross( p2, n2 ) + ( cross( n1, p2 ) ) * d ) / cross( p2, p3 ).norm();
                  q4 = ( cross( p2, n1 ) + ( cross( n2, p2 ) ) * d ) / cross( p2, p4 ).norm();
                  q2 = ( - ( cross( p3, n2 ) + ( cross( n1, p3 ) ) * d ) / cross( p2, p3 ).norm() ) - ( ( cross( p4, n1 ) + ( cross( n2, p4 ) ) * d ) / cross( p2, p4 ).norm() );
                  q1 = -q2 -q2 -q4;


                  // std::cout << "q1: " << q1 << " q2: " << q2 << " q3: " << q3 << " q4: " << q4 << std::endl;

                  double sumInvMasses = e->he->vertex->cMass + e->he->next->vertex->cMass + e->he->next->next->vertex->cMass + e->he->flip->next->next->vertex->cMass;
                  double sumQNorm2 = q1.norm2() + q2.norm2() + q3.norm2() + q4.norm2();
                  double mult;
                  if( sumInvMasses == 0. || sumQNorm2 == 0. ){
                     mult = 1.;
                  } else{
                     mult = sumQNorm2 * sumInvMasses;
                  }

                  double factor = - 4.0 * sqrt( 1 - d * d ) * ( acos( d ) - e->dihedralAngle ) / ( mult );
                  // std::cout << " d: " << d << " dihedralAngle: " << e->dihedralAngle << std::endl;
                  // std::cout << " sumInvMasses: " << sumInvMasses << " sumQNorm2: " << sumQNorm2 << " factor: " << factor <<  " mult: " << mult << std::endl;


                  e->he->vertex->estimate += e->he->vertex->cMass * q1 * factor;
                  e->he->next->vertex->estimate += e->he->next->vertex->cMass * q2 * factor;
                  e->he->next->next->vertex->estimate += e->he->next->next->vertex->cMass * q3 * factor;
                  e->he->flip->next->next->vertex->estimate += e->he->flip->next->next->vertex->cMass * q4 * factor;

                  // std::exit(EXIT_FAILURE);
               }
            }

            
            // for( VertexIter v = m_cloth->vertices.begin(); v != m_cloth->vertices.end(); ++v ){
            //    std::cout << "p: " << v->estimate << std::endl;   
            // }


         };

      protected:
      
      private:

         Mesh* m_cloth;
         double m_density; // kg / m^2

   };
}

#endif

