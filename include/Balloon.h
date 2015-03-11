// -----------------------------------------------------------------------------
// libDDG -- Balloon.h
// -----------------------------------------------------------------------------
//
// Collision Constraint where both objects are free to move/deform
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_BALLOON_H
#define DDG_BALLOON_H


#include "Vector.h"
#include "Vertex.h"
#include "Constraint.h"

namespace DDG
{

   class Balloon : public Constraint
   {
      public:

         Balloon( Mesh* closed_surface, double overPressure )
         {
            m_stiffness = 1.;
            m_equality = true;
            m_cardinality = closed_surface->faces.size();
            m_balloon = closed_surface;
            m_overPressure = overPressure;

            // compute original volume
            m_originalVolume = 0.;
            for( FaceIter f = m_balloon->faces.begin(); f != m_balloon->faces.end(); ++f ){
               m_originalVolume += dot( cross( f->he->vertex->position, f->he->next->vertex->position ), f->he->next->next->vertex->position );
            }
            std::cout << "m_originalVolume: " << m_originalVolume << std::endl;
         };

         virtual ~Balloon( void ){};

         virtual Vector forceFunction( Vector& position ){
            std::cerr << "forceFunction ( Vector& position ) not implemented for Constraint:: BALLOON " << std::endl;
            return Vector();
         }

         virtual void forceFunction( void )
         {

            double constraint_func = 0.;
            for( FaceIter f = m_balloon->faces.begin(); f != m_balloon->faces.end(); ++f ){
               constraint_func += dot( cross( f->he->vertex->estimate, f->he->next->vertex->estimate ), f->he->next->next->vertex->estimate );
            }
            std::cout << "(initial volume): " << constraint_func << std::endl;

            std::cout << "(goal volume): " << m_overPressure * m_originalVolume << std::endl;
            constraint_func += - m_overPressure * m_originalVolume;

            // std::cout << "difference: " << constraint_func << std::endl;
            if( constraint_func * constraint_func  < 1e-9 ){
               // std::cout << "equality constraint satisfied, returning" << std::endl;
               return;
            }

            double scaling_denom = 0.;
            for( VertexIter v = m_balloon->vertices.begin(); v != m_balloon->vertices.end(); ++v ){
               Vector gradient;
               HalfEdgeCIter h = v->he;
               do
               {
                  gradient += cross( h->next->vertex->estimate, h->next->next->vertex->estimate );
                  h = h->flip->next;
               }
               while( h != v->he );
               scaling_denom += v->invMass * gradient.norm2();
            }

            std::vector< Vector > delta_estimates;
            for( VertexIter v = m_balloon->vertices.begin(); v != m_balloon->vertices.end(); ++v ){
               Vector gradient;
               HalfEdgeCIter h = v->he;
               do
               {
                  gradient += cross( h->next->vertex->estimate, h->next->next->vertex->estimate );
                  h = h->flip->next;
               }
               while( h != v->he );

               // needs scaling factor and masses for weighing the results
               double scaling = constraint_func / scaling_denom;
               delta_estimates.push_back( - scaling * v->invMass * gradient * m_stiffness );
               // v->estimate -= scaling * v->invMass * gradient * m_stiffness;
            }

            unsigned count = 0;
            for( VertexIter v = m_balloon->vertices.begin(); v != m_balloon->vertices.end(); ++v ){
                v->estimate += delta_estimates[count];
                ++count;
            }


            // double volume = 0.;
            // for( FaceIter f = m_balloon->faces.begin(); f != m_balloon->faces.end(); ++f ){
            //    volume += dot( cross( f->he->vertex->estimate, f->he->next->vertex->estimate ), f->he->next->next->vertex->estimate );
            // }
            // std::cout << "(final volume): " << volume << std::endl << std::endl;

         };

      protected:
      
      private:

         Mesh* m_balloon;
         double m_overPressure;
         double m_originalVolume;

   };
}

#endif

