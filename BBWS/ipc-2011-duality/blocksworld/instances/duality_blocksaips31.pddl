(define (problem blocks-15-0)
(:domain blocks)
(:objects a c l d j h k o n g i f b m e - block)
(:init (on a c) (on a l) (on a d) (on a j) (on a h) (on a k) (on a o) (on a n) (on a g) (on a i) (on a f) (on a m) (on a e) (on c a) (on c l) (on c d) (on c j) (on c h) (on c k) (on c o) (on c n) (on c g) (on c i) (on c f) (on c b) (on c m) (on c e) (on l a) (on l c) (on l d) (on l h) (on l k) (on l o) (on l n) (on l g) (on l i) (on l f) (on l b) (on l m) (on l e) (on d a) (on d c) (on d l) (on d j) (on d h) (on d k) (on d o) (on d g) (on d i) (on d f) (on d b) (on d m) (on d e) (on j a) (on j c) (on j l) (on j h) (on j k) (on j o) (on j n) (on j g) (on j i) (on j f) (on j b) (on j m) (on j e) (on h a) (on h c) (on h l) (on h d) (on h j) (on h o) (on h n) (on h g) (on h i) (on h f) (on h b) (on h m) (on h e) (on k a) (on k c) (on k l) (on k d) (on k j) (on k h) (on k o) (on k n) (on k g) (on k i) (on k f) (on k b) (on k e) (on o a) (on o c) (on o l) (on o d) (on o j) (on o k) (on o n) (on o g) (on o i) (on o f) (on o b) (on o m) (on o e) (on n a) (on n c) (on n l) (on n d) (on n j) (on n h) (on n k) (on n o) (on n g) (on n f) (on n b) (on n m) (on n e) (on g a) (on g c) (on g l) (on g d) (on g j) (on g h) (on g k) (on g n) (on g i) (on g f) (on g b) (on g m) (on g e) (on i a) (on i l) (on i d) (on i j) (on i h) (on i k) (on i o) (on i n) (on i g) (on i f) (on i b) (on i m) (on i e) (on f a) (on f c) (on f l) (on f d) (on f j) (on f h) (on f k) (on f o) (on f n) (on f g) (on f i) (on f b) (on f m) (on b a) (on b c) (on b d) (on b j) (on b h) (on b k) (on b o) (on b n) (on b g) (on b i) (on b f) (on b m) (on b e) (on m a) (on m c) (on m l) (on m d) (on m j) (on m h) (on m k) (on m o) (on m n) (on m g) (on m i) (on m b) (on m e) (on e c) (on e l) (on e d) (on e j) (on e h) (on e k) (on e o) (on e n) (on e g) (on e i) (on e f) (on e b) (on e m) (ontable a) (ontable c) (ontable l) (ontable d) (ontable j) (ontable h) (ontable k) (ontable o) (ontable n) (ontable g) (ontable i) (ontable f) (ontable b) (ontable m) (ontable e) (clear a) (clear c) (clear l) (clear d) (clear j) (clear h) (clear k) (clear o) (clear n) (clear g) (clear i) (clear f) (clear b) (clear m) (clear e) (handempty) (holding a) (holding c) (holding l) (holding d) (holding j) (holding h) (holding k) (holding o) (holding n) (holding g) (holding i) (holding f) (holding b) (holding m) (holding e))
(:goal (and (on a c) (on a l) (on a d) (on a j) (on a h) (on a k) (on a n) (on a g) (on a i) (on a f) (on a b) (on a m) (on a e) (on c a) (on c l) (on c d) (on c j) (on c h) (on c k) (on c o) (on c n) (on c i) (on c f) (on c b) (on c m) (on c e) (on l a) (on l d) (on l j) (on l h) (on l k) (on l o) (on l n) (on l g) (on l i) (on l f) (on l b) (on l m) (on l e) (on d a) (on d c) (on d j) (on d h) (on d k) (on d o) (on d n) (on d g) (on d i) (on d f) (on d b) (on d m) (on d e) (on j a) (on j c) (on j l) (on j h) (on j k) (on j o) (on j n) (on j g) (on j i) (on j f) (on j b) (on j m) (on j e) (on h a) (on h c) (on h l) (on h d) (on h j) (on h k) (on h o) (on h n) (on h g) (on h i) (on h f) (on h b) (on h m) (on h e) (on k a) (on k c) (on k l) (on k d) (on k j) (on k h) (on k o) (on k n) (on k g) (on k i) (on k f) (on k b) (on k m) (on k e) (on o a) (on o c) (on o l) (on o d) (on o j) (on o h) (on o k) (on o n) (on o g) (on o i) (on o f) (on o b) (on o m) (on o e) (on n a) (on n c) (on n l) (on n d) (on n j) (on n h) (on n k) (on n o) (on n g) (on n i) (on n f) (on n b) (on n m) (on n e) (on g a) (on g c) (on g l) (on g d) (on g j) (on g h) (on g k) (on g o) (on g n) (on g i) (on g f) (on g b) (on g m) (on g e) (on i a) (on i c) (on i l) (on i d) (on i j) (on i k) (on i o) (on i n) (on i g) (on i f) (on i b) (on i m) (on i e) (on f a) (on f c) (on f l) (on f d) (on f j) (on f h) (on f o) (on f n) (on f g) (on f i) (on f b) (on f m) (on f e) (on b c) (on b l) (on b d) (on b j) (on b h) (on b k) (on b o) (on b n) (on b g) (on b i) (on b f) (on b m) (on b e) (on m a) (on m c) (on m l) (on m d) (on m j) (on m h) (on m k) (on m o) (on m g) (on m i) (on m f) (on m b) (on m e) (on e a) (on e c) (on e l) (on e d) (on e h) (on e k) (on e o) (on e n) (on e g) (on e i) (on e f) (on e b) (on e m) (ontable a) (ontable c) (ontable l) (ontable d) (ontable j) (ontable i) (ontable f) (ontable b) (ontable m) (ontable e) (clear a) (clear c) (clear l) (clear d) (clear j) (clear h) (clear k) (clear o) (clear n) (clear g) (holding a) (holding c) (holding l) (holding d) (holding j) (holding h) (holding k) (holding o) (holding n) (holding g) (holding i) (holding f) (holding b) (holding m) (holding e))))