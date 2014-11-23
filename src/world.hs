



module World where


type Position = (Double, Double)
type Velocity = (Double, Double)

type CircleData = (Position, Velocity, Double)

type SegmentData = (Position, Position, Double)

data Obstacle = Circle CircleData | Segment SegmentData

data World = World {
  robotSpeed :: Double,
  robot      :: Position,
  goal       :: CircleData,
  obstacles  :: [Obstacle]
  }







