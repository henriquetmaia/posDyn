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

         Balloon( void )
         {
            m_stiffness = 1.;
            m_equality = true;
            m_cardinality = mesh_size;

         };

         virtual ~Balloon( void );

         virtual double forceFunction( vector< Vertex* > vertices, unsigned& numIters )
         {
            // needs scaling factor and masses for weighing the results 
         };

      protected:
      
      private:

         Vector m_pressure;
         Vector m_originalVolume;

   };
}

#endif

