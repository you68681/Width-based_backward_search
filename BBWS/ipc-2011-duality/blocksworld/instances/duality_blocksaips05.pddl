(define (problem blocks-5-1)
(:domain blocks)
(:objects a d c e b - block)
(:init (on a d) (on a c) (on a b) (on d a) (on d e) (on d b) (on c a) (on c d) (on c e) (on e a) (on e d) (on e c) (on e b) (on b d) (on b c) (on b e) (ontable a) (ontable d) (ontable c) (ontable e) (ontable b) (clear a) (clear d) (clear c) (clear e) (clear b) (handempty) (holding a) (holding d) (holding c) (holding e) (holding b))
(:goal (and (on a c) (on a e) (on a b) (on d a) (on d c) (on d e) (on d b) (on c a) (on c d) (on c e) (on c b) (on e a) (on e d) (on e c) (on e b) (on b d) (on b c) (on b e) (ontable a) (ontable b) (clear a) (clear d) (holding a) (holding d) (holding c) (holding e) (holding b))))