

module Vec2 where


apply f (ax, ay) (bx, by) = (f ax bx, f ay by)

add a b = apply (+) a b
sub a b = apply (-) a b

scale s (x, y) = (s * x, s * y)

dot (ax, ay) (bx, by) = ax*bx + ay*by

length v = sqrt $ dot v v

normalize v = scale (1 / Vec2.length v) v

