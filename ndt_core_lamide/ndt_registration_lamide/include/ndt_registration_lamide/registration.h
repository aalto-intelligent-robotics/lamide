#ifndef REGISTRATION_H
#define REGISTRATION_H
typedef enum regstatus{ SUCCESS=0, DELTA_SCORE_LIMIT=1, GRADIENT_VANISHED=2, MAX_ITR=3, WRONG_DIRECTION=4, FAILURE=5 }regStatus;
/* SUCCESS = registration success due to any condition
 * DELTA_SCORE_LIMIT = registration converged after delta score < th
 * GRADIENT_VANISHED = converged, norm of gradient is small
 * MAX_ITR = No convergence after max interations has been reached
 * WRONG_DIRECTION = Moving in the direction of the gradient didn't improve score
 * FAILURE = Unclassified major problem (unreliable)
 * */

#endif // REGISTRATION_H
