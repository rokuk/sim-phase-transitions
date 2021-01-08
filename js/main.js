// module aliases
let Engine = Matter.Engine,
    Render = Matter.Render,
    World = Matter.World,
    Bodies = Matter.Bodies,
    Body = Matter.Body,
    MouseConstraint = Matter.MouseConstraint,
    Mouse = Matter.Mouse,
    SAT = Matter.SAT,
    Runner = Matter.Runner,
    Vector = Matter.Vector,
    Events = Matter.Events,
    Composite = Matter.Composite;

let debug = false;
let isDown;
let downBody;
let cnv;
let ctx;
let epsilonf = 1 / 24;

function defaultFor(arg, val) {
    return typeof arg !== "undefined" ? arg : val;
}

function gen(dolzina, pad) {
    return dolzina * (Math.random() * 0.95 + 0.025 + pad);
}

function collides(x, y, r, circles) {
    for (let circle of circles) {
        if (
            Math.pow(x - circle.position.x, 2) +
                Math.pow(y - circle.position.y, 2) <
            4 * Math.pow(r, 2)
        ) {
            return true;
        }
    }
    return false;
}

function sigmaf(radij) {
    return Math.pow(radij * 1.1, 2);
}

function oldLennardJonesAttractor(bodyA, bodyB) {
    let sigma2 =
        Math.pow(radij / 1.12246204831, 2) * defaultFor(options.sigmaf, 1);
    let scale = defaultFor(options.scale, 140);
    let dx = bodyA.position.x - bodyB.position.x;
    let dy = bodyA.position.y - bodyB.position.y;
    let dist2 = Math.pow(dx, 2) + Math.pow(dy, 2);
    let term = sigma2 / dist2;
    let velikostd2 = (2 * Math.pow(term, 3) - term) / Math.pow(dist2, 2);
    let force = {
        x: velikostd2 * dx * scale,
        y: velikostd2 * dy * scale,
    };
    Body.applyForce(bodyB, bodyB.position, Vector.neg(force));
    Body.applyForce(bodyA, bodyA.position, force);
}

function generateCircles(
    N,
    width,
    height,
    radij,
    attract,
    doFillCircle,
    options
) {
    let fillStyle = defaultFor(options.fillStyle, "#ffcc66");
    let pad = defaultFor(options.pad, 0);
    let circles = [];

    for (let i = 0; i < N; i++) {
        let x, y;

        // prevent generating collided circles
        do {
            x = gen(width, pad);
            y = gen(height, pad);
        } while (collides(x, y, radij, circles));

        let options = {
            inertia: Infinity,
            friction: 0,
            frictionAir: 0,
            frictionStatic: 0,
            restitution: 1,
        };

        // fill circle with specified style if given, if not default color
        if (doFillCircle) {
            options.render = {};
            options.render.fillStyle = fillStyle;
        }

        // attract with potential or no attraction
        if (attract) {
            let sigma2 = sigmaf(radij);
            options.plugin = {};
            options.plugin.attractors = [
                function (bodyA, bodyB) {
                    let dx = bodyA.position.x - bodyB.position.x;
                    let dy = bodyA.position.y - bodyB.position.y;
                    let dist2 = Math.pow(dx, 2) + Math.pow(dy, 2);
                    let term = sigma2 / dist2;
                    let velikostd2 =
                        (Math.pow(term, 3) / dist2) *
                        (1 - 2 * Math.pow(term, 3)) *
                        24 *
                        epsilonf;
                    let force = {
                        x: velikostd2 * dx,
                        y: velikostd2 * dy,
                    };
                    Body.applyForce(bodyA, bodyA.position, Vector.neg(force));
                    Body.applyForce(bodyB, bodyB.position, force);
                },
            ];
        }
        let circ = Bodies.circle(x, y, radij, options, 100);
        circles.push(circ);
    }
    return circles;
}

function applyF(downBody, event) {
    Body.applyForce(
        downBody,
        {
            x: downBody.position.x,
            y: downBody.position.y,
        },
        {
            x:
                (event.source.mouse.position.x - downBody.position.x) * 1e-5 -
                downBody.velocity.x * 1e-4,
            y:
                (event.source.mouse.position.y - downBody.position.y) * 1e-5 -
                downBody.velocity.y * 1e-4,
        }
    );
}

function mouseDown(event) {
    isDown = true;
    downBody = event.source.body;
}

function mouseUp(event) {
    isDown = false;
    if (downBody !== null) {
        applyF(downBody, event);
    }
}

function afterRender(event) {
    if (isDown && downBody !== null) {
        ctx.beginPath();
        ctx.moveTo(downBody.position.x, downBody.position.y);
        ctx.lineTo(
            event.source.mouse.position.x,
            event.source.mouse.position.y
        );
        ctx.strokeStyle = "#00FF66";
        ctx.lineWidth = 3;
        ctx.stroke();
        applyF(downBody, event);
    }
}

function createSim(elid, width, height, doMouse) {
    let canv = document.getElementById(elid);
    ctx = canv.getContext("2d");

    width = defaultFor(width, 500);
    height = defaultFor(height, 500);
    doMouse = defaultFor(doMouse, true);
    canv.width = width;
    canv.height = height;

    // create an engine
    let engine = Engine.create({
        enableSleeping: false,
        positionIterations: 50,
        velocityIterations: 50,
        constraintIterations: 50,
    });
    let world = engine.world;

    // gravity
    world.gravity.y = 0;

    // create a renderer
    let render = Render.create({
        canvas: canv,
        engine: engine,
        options: {
            wireframes: debug,
            height: height,
            width: width,
            showSleeping: debug,
            background: "#222222",
        },
    });

    Matter.Resolver._restingThresh = 0.0001; // če sistem izgublja energijo zmanjšaj to število

    // border box
    let borderOptions = {
        isStatic: true,
        friction: 0,
        frictionStatic: 0,
        restitution: 1,
        render: {
            fillStyle: "#666666",
        },
    };
    let border1 = Bodies.rectangle(width / 2, -20, width, 50, borderOptions); // top
    let border2 = Bodies.rectangle(
        width / 2,
        height + 20,
        width,
        50,
        borderOptions
    ); // bottom
    let border3 = Bodies.rectangle(-20, height / 2, 50, height, borderOptions); // left
    let border4 = Bodies.rectangle(
        width + 20,
        height / 2,
        50,
        height,
        borderOptions
    ); // right
    World.add(world, [border1, border2, border3, border4]);

    // mouse stuff
    if (doMouse) {
        let mouse = Mouse.create(render.canvas);
        let mouseConstraint = MouseConstraint.create(engine, {
            mouse: mouse,
            constraint: {
                stiffness: 0,
                render: {
                    visible: false,
                },
            },
        });
        World.add(world, mouseConstraint);
        render.mouse = mouse;
        mcref = mouseConstraint;
        Events.on(mouseConstraint, "mousedown", mouseDown);
        Events.on(mouseConstraint, "mouseup", mouseUp);
    }

    Render.lookAt(render, {
        min: { x: 0, y: 0 },
        max: { x: width, y: height },
    });
    Events.on(render, "afterRender", afterRender);

    return { engine, render, world};
}

function kineticnaEnergija(world) {
    let bodiescomp = Composite.allBodies(world);
    let vsotaken = 0;
    for (let bod of bodiescomp) {
        if (!bod.isStatic) {
            vsotaken += bod.mass * Math.pow(bod.speed, 2);
        }
    }
    return vsotaken / 2;
}

function potencialnaEnergija(world, radij) {
    let bodiescomp = Composite.allBodies(world);
    let sigma2 = sigmaf(radij);
    let vsotapen = 0;
    for (let bod1 of bodiescomp) {
        if (!bod1.isStatic) {
            for (let bod2 of bodiescomp) {
                if (!bod2.isStatic && bod1 !== bod2) {
                    let term =
                        sigma2 /
                        (Math.pow(bod1.position.x - bod2.position.x, 2) +
                            Math.pow(bod1.position.y - bod2.position.y, 2));
                    vsotapen += Math.pow(term, 6) - Math.pow(term, 3);
                }
            }
        }
    }
    return (vsotapen / 2) * 4 * epsilonf;
}

function celotnaEnergija(world, radij) {
    return kineticnaEnergija(world) + potencialnaEnergija(world, radij);
}

function changeTemperature(world, delez) {
    let bodiescomp = Composite.allBodies(world);
    for (let bod of bodiescomp) {
        if (!bod.isStatic) {
            Body.setVelocity(bod, {
                x: bod.velocity.x * delez,
                y: bod.velocity.y * delez,
            });
        }
    }
}
