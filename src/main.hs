

--{-# LANGUAGE NoImplicitPrelude #-}

module Main where


import Graphics.UI.GLUT as GLUT
import Control.Monad
import Data.IORef as IORef
import Data.List  as List
import Debug.Trace as Trace

import World
import Vec2


robotRadius = 0.2 :: Double

gridSize = 20 :: Double

t s = 2 * s / gridSize - 1


updateInterval = 30 :: Int -- in ms

dt :: Double
dt = 1 / fromIntegral updateInterval


color3f r g b = color $ Color3 (realToFrac r) (realToFrac g) (realToFrac (b :: Double) :: GLdouble)
vertex3f x y z = vertex $ Vertex3 (realToFrac x) (realToFrac y) (realToFrac (z :: Double) :: GLdouble)

drawCircle paint (cx, cy) inRadius = renderPrimitive TriangleFan $ do
  color paint
  mapM_
    (\ (x, y) -> vertex3f x y 0)
    (fmap (\ o -> (cx + r * sin o, cy + r * cos o)) (fmap (2 * pi / numSegments *) [0..numSegments]))
  where
    r = 2 * inRadius / gridSize
    numSegments = 20 :: Double

drawLine ((x1, y1), (x2, y2)) = do
  vertex3f x1 y1 0
  vertex3f x2 y2 0

drawSegment paint (x1, y1) (x2, y2) w = do
  renderPrimitive Quads ( do
    color paint
    vertex3f (t ax) (t ay) 0
    vertex3f (t bx) (t by) 0
    vertex3f (t cx) (t cy) 0
    vertex3f (t dx) (t dy) 0)
  drawCircle paint (t x1, t y1) w
  drawCircle paint (t x2, t y2) w
  where
    dir  = Vec2.normalize $ Vec2.sub (x2, y2) (x1, y1)
    perp = (- snd dir, fst dir)
    (ax, ay) = Vec2.add (x1, y1) (Vec2.scale   w  perp)
    (bx, by) = Vec2.add (x2, y2) (Vec2.scale   w  perp)
    (cx, cy) = Vec2.add (x2, y2) (Vec2.scale (-w) perp)
    (dx, dy) = Vec2.add (x1, y1) (Vec2.scale (-w) perp)

-- [0..gridSize] -> [-1..1]
scaleForDrawing (x, y) = (t x, t y)
scalePairForDrawing (a, b) = (scaleForDrawing a, scaleForDrawing b)


drawGrid w = do
  mapM_
    (\ p -> drawCircle (Color4 0.7 0.7 0.7 (1.0 :: GLfloat)) (scaleForDrawing p) 0.05)
    centres
  renderPrimitive Lines $ do
    color3f 0.7 0.7 0.7
--    mapM_
--      (drawLine . scalePairForDrawing)
--      (List.zip (List.zip (List.repeat 0.0) [0..gridSize]) (List.zip (List.repeat gridSize) [0..gridSize]))
--    mapM_
--      (drawLine . scalePairForDrawing)
--      (List.zip (List.zip [0..gridSize] (List.repeat 0.0)) (List.zip [0..gridSize] (List.repeat gridSize)))
    mapM_
      (drawLine . scalePairForDrawing)
      (List.zip centres (fmap (\ c -> Vec2.add c (Vec2.scale 0.5 (forceDirection w c))) centres))
  where
    numbers = fmap (+ 0.5) [0..gridSize]
    centres = [(x, y) | x <- numbers, y <- numbers]

obstacleColor = Color3 0.8 0.2 (0.2 :: GLdouble)

drawObstacle (Circle (p, _, r)) = drawCircle obstacleColor (scaleForDrawing p) (r - robotRadius)
drawObstacle (Segment (p1, p2, w)) = drawSegment obstacleColor p1 p2 (w - robotRadius)

drawObstacles obstacles = mapM_ drawObstacle obstacles

drawWorld w = do
    drawObstacles (World.obstacles w)
    drawCircle (Color3 0.2 0.8 (0.2 :: GLdouble)) (scaleForDrawing pg) gr
    drawCircle (Color3 0.8 0.8 (0.2 :: GLdouble)) (scaleForDrawing pr) robotRadius
  where
    (pg, _, gr) = World.goal w
    pr = World.robot w

updateCircle (p, v, r)
  | x < 0        = ((-x, y), Vec2.scale (-1) v, r)
  | x > gridSize = ((2*gridSize - x, y), Vec2.scale (-1) v, r)
  | otherwise    = ((x, y), v, r)
  where
    (x, y) = Vec2.add p (Vec2.scale dt v)

updateObstacle (Circle  x) = Circle (updateCircle x)
updateObstacle (Segment x) = Segment x

updateObstacles w = fmap updateObstacle (World.obstacles w)

segmentForce (a, b, w) p
  | Vec2.dot ab ap < 0 = Vec2.scale (w^2 / apDist^3) ap
  | Vec2.dot ab bp > 0 = Vec2.scale (w^2 / bpDist^3) bp
  | otherwise = Vec2.scale (-w^2 / perpDist^2) perp
  where
    ab = Vec2.sub b a
    ap = Vec2.sub p a
    bp = Vec2.sub p b
    apDist = Vec2.length ap
    bpDist = Vec2.length bp
    perp = Vec2.normalize (- snd ab, fst ab)
    perpDist = Vec2.dot perp ap

forceDirection :: World -> (Double, Double) -> (Double, Double)
forceDirection w pos = Vec2.normalize (Vec2.add fG fO)
  where
    obstacleForce (Circle (po, _, r)) = Vec2.scale (r^2 / length^3) dif
      where
        dif    = Vec2.sub pos po
        length = Vec2.length dif
    obstacleForce (Segment s) = segmentForce s pos
    (goal, _, _) = World.goal w
    fG   = Vec2.normalize (Vec2.sub goal pos)
    fO   = List.foldl Vec2.add (0, 0) (fmap obstacleForce (World.obstacles w))

updateGoal w
  | x < 0        = ((-x, y), Vec2.scale (-1) v, r)
  | x > gridSize = ((2*gridSize - x, y), Vec2.scale (-1) v, r)
  | otherwise    = ((x, y), v, r)
  where
    (p, v, r) = World.goal w
    (x, y)    = Vec2.add p (Vec2.scale dt v)

updateRobot w
  | Vec2.length (Vec2.sub p g) < r = (2, 2)
  | otherwise = Vec2.add p (Vec2.scale (dt * World.robotSpeed w) (forceDirection w p))
  where
    (g, _, r) = World.goal w
    p = World.robot w

updateWorld :: World -> World
updateWorld w = World
  (World.robotSpeed w)
  (updateRobot w)
  (updateGoal w)
  (updateObstacles w)

staticWorldA = World 6 (1, 1) ((19, 19), (0, 0), 1) []
staticWorldB = World 6 (1, 1) ((19, 19), (0, 0), 1) [Circle ((11, 13), (0, 0), 3)]
staticWorldC = World 6 (1, 1) ((19, 19), (0, 0), 1) [Circle ((11, 13), (0, 0), 3) , Circle ((7, 5), (0, 0), 3)]
staticWorldD = World 6 (1, 1) ((19, 19), (0, 0), 1) [Circle ((4, 11.5), (0, 0), 2), Circle((7, 5), (0, 0), 3)]
staticWorldE = World 6 (1, 1) ((19, 19), (0, 0), 1) [Circle ((4, 11.5), (0, 0), 2), Circle((7, 5), (0, 0), 3), Circle ((11, 16), (0, 0), 2.5)]

movingGoalA = World 6 (1, 1) ((19, 19), (-8, 0), 1) []
movingGoalB = World 6 (1, 1) ((19, 19), (-3, 0), 1) [Circle ((4, 11.5), (0, 0), 2), Circle ((7, 5), (0, 0), 3), Circle ((11, 16), (0, 0), 2.5)]

dynamicWorldA = World 6 (1, 1) ((19, 19), (-3, 0), 1) [Circle ((4, 11.5), (4, 0), 2), Circle ((7, 5), (2, 0), 3), Circle ((11, 16), (-2, 0), 2.5)]
dynamicWorldB = World 6 (1, 1) ((19, 19), (-3, 0), 1) [Circle ((4, 11.5), (4, 0), 2), Circle ((7, 5), (2, 0), 3), Circle ((11, 16), (-2, 0), 2.5), Segment ((3, 7), (7, 3), 1)]


main :: IO ()
main = do
  (programName, arguments) <- getArgsAndInitialize
  initialDisplayMode       $= [DoubleBuffered]
  initialWindowPosition    $= Position 100 50
  initialWindowSize        $= Size 640 640
  window                   <- createWindow "Simple Field Navigation"
  count                    <- newIORef 0
  time                     <- get elapsedTime
  lastTime                 <- newIORef time
  world                    <- newIORef dynamicWorldB
  displayCallback          $= display count world
  idleCallback             $= Just (idle lastTime count world)
  clearColor               $= Color4 0.2 0.2 0.2 1.0
  GLUT.mainLoop

display :: IORef Int -> IORef World -> DisplayCallback
display count world = do
  clear [ ColorBuffer ]
  c <- get count
  w <- get world
  drawGrid w
  drawWorld w
  swapBuffers

idle :: IORef Int -> IORef Int -> IORef World -> IdleCallback
idle lastTime count world = do
  time <- get elapsedTime
  last <- get lastTime
  when (time - last > updateInterval) $ do
    modifyIORef' count    (+ 1)
    modifyIORef' lastTime (+ updateInterval)
    modifyIORef' world    updateWorld
  postRedisplay Nothing

























