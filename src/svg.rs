use ::svg::parser::Event;
use svg::node::element::path::{Command, Data};

pub fn read_svg(path: &str) {
    let mut content = String::new();
    for event in svg::open(path, &mut content).unwrap() {
        if let Event::Tag(_, _, attributes) = event
            && let Some(data) = attributes.get("d")
        {
            let data = Data::parse(data).unwrap();
            for command in data.iter() {
                match command {
                    Command::Move(pos, params) => println!("Move: {:?}, {:?}", pos, params),
                    Command::Line(pos, params) => println!("Line: {:?}, {:?}", pos, params),
                    Command::HorizontalLine(position, parameters) => {
                        println!("HorizontalLine: {:?}, {:?}", position, parameters)
                    }
                    Command::VerticalLine(position, parameters) => {
                        println!("VerticalLine: {:?}, {:?}", position, parameters)
                    }
                    Command::QuadraticCurve(position, parameters) => {
                        println!("QuadraticCurve: {:?}, {:?}", position, parameters)
                    }
                    Command::SmoothQuadraticCurve(position, parameters) => {
                        println!("SmoothQuadraticCurve: {:?}, {:?}", position, parameters)
                    }
                    Command::CubicCurve(position, parameters) => {
                        println!("CubicCurve: {:?}, {:?}", position, parameters)
                    }
                    Command::SmoothCubicCurve(position, parameters) => {
                        println!("SmoothCubicCurve: {:?}, {:?}", position, parameters)
                    }
                    Command::EllipticalArc(position, parameters) => {
                        println!("EllipticalArc: {:?}, {:?}", position, parameters)
                    }
                    Command::Close => {
                        println!("Close")
                    }
                }
            }
        }
    }
}
