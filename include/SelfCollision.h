// -----------------------------------------------------------------------------
// libDDG -- SelfCollision.h
// -----------------------------------------------------------------------------
//
// Self Collision Constraint where object (cloth) collides with itself and needs thickness
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_SELF_COLLISION_H
#define DDG_SELF_COLLISION_H


#include "Vector.h"
#include "Vertex.h"
#include "Constraint.h"

namespace DDG
{

   class SelfCollision : public Constraint
   {
      public:

         SelfCollision( void )
         {
            m_stiffness = 1.;
            m_equality = false;
            m_cardinality = 4;

         };

         virtual ~SelfCollision( void );

         virtual double forceFunction( vector< Vertex* > vertices, unsigned& numIters )
         {
            //check if function is less than zero, else skip

            // // or? check if initial position is already inside the object
            // do continous time detection to find collision point
            // check if completely inside the object
            // if so project to surface
         };

      protected:
      
      private:

         double m_thickness;
         Vector m_contactPoint;
         Vector m_contactNormal;

   };
}

#endif

