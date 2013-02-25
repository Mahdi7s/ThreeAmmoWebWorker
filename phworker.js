// Author: www.mahdi7s.com

"use strict";

importScripts("ammo.js");

self.ex = {
    toBt: function (v) {
        return new Ammo.btVector3(v.x, v.y, v.z);
    },
    toBtQuat: function (v) {
        var yaw = v.y, pitch = v.x, roll = v.z;
        return new Ammo.btQuaternion(yaw, pitch, roll);
    }
};

self.init = function () {
    var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration(),
    dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration),
    overlappingPairCache = new Ammo.btDbvtBroadphase(),
    solver = new Ammo.btSequentialImpulseConstraintSolver();

    self.world = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    self.world.setGravity(new Ammo.btVector3(0, -20, 0));

    self.bodies = [];
};

self.simulate = function (data) {
    self.world.stepSimulation(data.dt, // time step
                              Math.ceil(data.dt / (1 / 60)), // max sub steps
                              1 / 60); // fixed time step
    var toUpdateMeshes = [];
    self.bodies.forEach(function (body) {

        var transform = new Ammo.btTransform();
        body.getMotionState().getWorldTransform(transform);
        var origin = transform.getOrigin(),
            rotation = transform.getRotation();

        toUpdateMeshes.push({ meshName: body.meshName, pos: { x: origin.x(), y: origin.y(), z: origin.z() }, quat: { x: rotation.x(), y: rotation.y(), z: rotation.z(), w: rotation.w() } });
    });
    self.postMessage({ task: "syncMesh", meshes: toUpdateMeshes });
};

self.addObjectToSimulationWorld = function (desc) {
    var shape, body, motionState, transform, localInertia, rbInfo;

    transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(self.ex.toBt(desc.pos));
    transform.setRotation(self.ex.toBtQuat(desc.rot));
    motionState = new Ammo.btDefaultMotionState(transform);
    localInertia = new Ammo.btVector3(0, 0, 0);

    switch (desc.shape) {
        case "concave":
            var compoundShape = new Ammo.btCompoundShape(true);
            var identityTrasform = new Ammo.btTransform();
            identityTrasform.setIdentity();

            desc.info.forEach(function (c, i, arr) {
                var convex = new Ammo.btConvexHullShape();
                convex.addPoint(self.ex.toBt(c[0]));
                convex.addPoint(self.ex.toBt(c[1]));
                convex.addPoint(self.ex.toBt(c[2]));
                c[3] && convex.addPoint(self.ex.toBt(c[3]));

                compoundShape.addChildShape(identityTrasform, convex);
            });
            var mass = desc.mass || 0;
            compoundShape.calculateLocalInertia(mass, localInertia);
            rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, compoundShape, localInertia);
            rbInfo.set_m_friction(desc.friction || 0.4);
            rbInfo.set_m_restitution(desc.restitution || 0.2);
            body = new Ammo.btRigidBody(rbInfo);
            self.world.addRigidBody(body);
            break;
        case "sphere":
            var radius = desc.radius;
            shape = new Ammo.btSphereShape(radius);
            var mass = desc.mass || (4 / 3) * Math.PI * (Math.pow(radius, 3));
            shape.calculateLocalInertia(mass, localInertia);
            rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, shape, localInertia);
            rbInfo.set_m_friction(desc.friction || 0.4);
            rbInfo.set_m_restitution(desc.restitution || 0.2);
            body = new Ammo.btRigidBody(rbInfo);
            self.world.addRigidBody(body);
            break;
    }
    body.meshName = desc.meshName;
    self.bodies.push(body);
};

self.onmessage = function (evt) {
    switch (evt.data.task) {
        case "init":
            self.init();
            break;
        case "addObjectToSimulationWorld":
            self.addObjectToSimulationWorld(evt.data);
            break;
        case "simulate":
            self.simulate(evt.data);
            break;
    }
};
