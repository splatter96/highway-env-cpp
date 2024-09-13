#ifndef LANE_H
#define LANE_H

#include <array>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <memory>

#include <Eigen/Dense>

typedef Eigen::Vector2d Vector;

#define VEHICLE_LENGTH 5
#define DEFAULT_WIDTH 4

/*"A lane side line type.*/
enum LineType{
    NONE = 0,
    STRIPED = 1,
    CONTINUOUS = 2,
    CONTINUOUS_LINE = 3
};

class AbstractLane {
public:
        float length;
        bool forbidden;
        Vector start;
        Vector end;
        std::array<LineType, 2> line_types;

        AbstractLane(Vector start, Vector end){
            this->start = start;
            this->end = end;
        };

        virtual ~AbstractLane() = default;

        virtual std::unique_ptr<AbstractLane> clone() const = 0;

        /*
        Convert local lane coordinates to a world position.

        :param longitudinal: longitudinal lane coordinate [m]
        :param lateral: lateral lane coordinate [m]
        :return: the corresponding world position [m]
        */
        virtual Vector position(float longitudinal, float lateral) = 0;

        /*
        Convert a world position to local lane coordinates.

        :param position: a world position [m]
        :return: the (longitudinal, lateral) lane coordinates [m]
        */
        virtual Vector local_coordinates(Vector pos) = 0;

        /*
        Get the lane heading at a given longitudinal lane coordinate.

        :param longitudinal: longitudinal lane coordinate [m]
        :return: the lane heading [rad]
        */
        virtual float heading_at(float longitudinal) = 0;

        /*
        Get the lane width at a given longitudinal lane coordinate.

        :param longitudinal: longitudinal lane coordinate [m]
        :return: the lane width [m]
        */
        virtual float width_at(float longitudinal) = 0;

        /*
        Whether a given world position is on the lane.

        :param position: a world position [m]
        :param longitudinal: (optional) the corresponding longitudinal lane coordinate, if known [m]
        :param lateral: (optional) the corresponding lateral lane coordinate, if known [m]
        :param margin: (optional) a supplementary margin around the lane width
        :return: is the position on the lane?
        */
        //bool on_lane(Vector position, float longitudinal, float lateral, float margin){
        bool on_lane(Vector position, float margin){
            //if longitudinal is None or lateral is None:
                //longitudinal, lateral = this->local_coordinates(position)

            float longitudinal = this->local_coordinates(position).x();
            float lateral = this->local_coordinates(position).y();
            bool is_on = fabs(lateral) <= this->width_at(longitudinal) / 2 + margin &&  -VEHICLE_LENGTH <= longitudinal && longitudinal < this->length + VEHICLE_LENGTH;

            return is_on;
        }

        /*
        Whether the lane is reachable from a given world position

        :param position: the world position [m]
        :return: is the lane reachable?
        */
        bool is_reachable_from(Vector position){
            if(this->forbidden)
                return false;

            float longitudinal = this->local_coordinates(position).x();
            float lateral = this->local_coordinates(position).y();
            return fabs(lateral) <= 2. * this->width_at(longitudinal) && 0. <= longitudinal && longitudinal < this->length + VEHICLE_LENGTH;

        }

        bool after_end(Vector position){
            float longitudinal = this->local_coordinates(position).x();
            return longitudinal > this->length - (10. + VEHICLE_LENGTH / 2.);
        }

        /*
        Compute the L1 distance [m] from a position to the lane.
        */
        float distance(Vector position){
            std::cout << "called distance" << std::endl;
            Vector local_pos  = this->local_coordinates(position);
            return fabs(local_pos.y()) + fmax(local_pos.x() - this->length, 0.) + fmax(0. - local_pos.x(), 0.);
        }

        /*
        Compute a weighted distance in position and heading to the lane.
        */
        float distance_with_heading(Vector position, float heading){
            Vector coords = this->local_coordinates(position);
            float s = coords.x();
            float r = coords.y();
            return fabs(r) + fmax(s - length, 0) + fmax(-s, 0);
        }
};


/*A lane going in straight line.*/
class StraightLane : public AbstractLane{
public:
    Vector direction;
    Vector direction_lateral;
    float width;
    float heading;

    StraightLane(Vector start,
                 Vector end,
                 float width,
                 std::array<LineType, 2> line_types,
                 bool forbidden
                 ) : AbstractLane(start, end){
        /*
        New straight lane.

        :param start: the lane starting position [m]
        :param end: the lane ending position [m]
        :param width: the lane width [m]
        :param line_types: the type of lines on both sides of the lane
        :param forbidden: is changing to this lane forbidden
        */

        this->width = width;
        this->heading = atan2(this->end.y() - this->start.y(), this->end.x() - this->start.x());
        this->length = (this->end - this->start).norm();
        this->line_types = line_types;
        this->direction = (this->end - this->start) / this->length;
        this->direction_lateral << -this->direction.y(), this->direction.x();
        this->forbidden = forbidden;
    }

     virtual std::unique_ptr<AbstractLane> clone() const override {
         return std::make_unique<StraightLane>(*this);
    }

    Vector position(float longitudinal, float lateral) override {
        return this->start + longitudinal * this->direction + lateral * this->direction_lateral;
    }

    float heading_at(float longitudinal) override {
        return this->heading;
    }

    float width_at(float longitudinal) override {
        std::cout << "called width at" << std::endl;
        return this->width;
    }

    Vector local_coordinates(Vector position){
        Vector delat = position - this->start;

        float longitudinal = delat.dot(this->direction);
        float lateral = delat.dot(this->direction_lateral);

        Vector p;
        p << longitudinal, lateral;

        return p;
    }
};

/*A sinusoidal lane. */
class SineLane : public StraightLane{
public:
    float amplitude;
    float pulsation;
    float phase;

    SineLane(Vector start,
                 Vector end,
                 float amplitude,
                 float pulsation,
                 float phase,
                 float width,
                 std::array<LineType, 2>line_types,
                 bool forbidden = false) : StraightLane(start, end, width, line_types, forbidden){
        /*
        New sinusoidal lane.

        :param start: the lane starting position [m]
        :param end: the lane ending position [m]
        :param amplitude: the lane oscillation amplitude [m]
        :param pulsation: the lane pulsation [rad/m]
        :param phase: the lane initial phase [rad]
        */
        this->amplitude = amplitude;
        this->pulsation = pulsation;
        this->phase = phase;
    }

     virtual std::unique_ptr<AbstractLane> clone() const override {
         return std::make_unique<SineLane>(*this);
    }


    Vector position(float longitudinal, float lateral){
        return StraightLane::position(longitudinal, lateral + this->amplitude * sin(this->pulsation * longitudinal + this->phase));
    }

    float heading_at(float longitudinal){
        return StraightLane::heading_at(longitudinal) + atan(this->amplitude * this->pulsation * cos(this->pulsation * longitudinal + this->phase));
    }

    Vector local_coordinates(Vector position){
        Vector loc_pos = StraightLane::local_coordinates(position);
        float longitudinal = loc_pos.x();
        float lateral = loc_pos.y();

        Vector p;
        p << longitudinal, lateral - this->amplitude * sin(this->pulsation * longitudinal + this->phase);
        return p;
    }
};

//class CircularLane(AbstractLane):

    //"""A lane going in circle arc."""

    //def __init__(self,
                 //center: Vector,
                 //radius: float,
                 //start_phase: float,
                 //end_phase: float,
                 //clockwise: bool = True,
                 //width: float = DEFAULT_WIDTH,
                 //line_types: Tuple[LineType] = None,
                 //forbidden: bool = False,
                 //speed_limit: float = 20,
                 //priority: int = 0) -> None:
        //super().__init__()
        //self.center = np.array(center)
        //self.radius = radius
        //self.start_phase = start_phase
        //self.end_phase = end_phase
        //self.direction = 1 if clockwise else -1
        //self.width = width
        //self.line_types = line_types or [LineType.STRIPED, LineType.STRIPED]
        //self.forbidden = forbidden
        //self.length = radius*(end_phase - start_phase) * self.direction
        //self.priority = priority
        //self.speed_limit = speed_limit

    //def position(self, longitudinal: float, lateral: float) -> np.ndarray:
        //phi = self.direction * longitudinal / self.radius + self.start_phase
        //return self.center + (self.radius - lateral * self.direction)*np.array([np.cos(phi), np.sin(phi)])

    //def heading_at(self, longitudinal: float) -> float:
        //phi = self.direction * longitudinal / self.radius + self.start_phase
        //psi = phi + np.pi/2 * self.direction
        //return psi

    //def width_at(self, longitudinal: float) -> float:
        //return self.width

    //def local_coordinates(self, position: np.ndarray) -> Tuple[float, float]:
        //delta = position - self.center
        //phi = np.arctan2(delta[1], delta[0])
        //phi = self.start_phase + utils.wrap_to_pi(phi - self.start_phase)
        //r = np.linalg.norm(delta)
        //longitudinal = self.direction*(phi - self.start_phase)*self.radius
        //lateral = self.direction*(self.radius - r)
        //return longitudinal, lateral

#endif // LANE_H
