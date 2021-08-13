package com.github.programmer3481.easybullet;

import java.util.HashMap;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.btConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btDiscreteDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.dynamics.btSequentialImpulseConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody.btRigidBodyConstructionInfo;
import com.badlogic.gdx.physics.bullet.linearmath.btDefaultMotionState;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import com.badlogic.gdx.physics.bullet.linearmath.btTransform;

public class EasyBullet {
    private btDynamicsWorld world;
    private btDispatcher dispatcher;
    private btCollisionConfiguration collisionConfig;
    private btBroadphaseInterface broadphase;
    private btConstraintSolver solver;

    private HashMap<String, RigidBody> objectList;

    public EasyBullet() {
        objectList = new HashMap<>();

        Bullet.init();

        collisionConfig = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfig);
        broadphase = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver();
        world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
        world.setGravity(new Vector3(0.0f, -9.8f, 0.0f));
    }

    private static class RigidBody {
        private btCollisionShape shape;
        private btMotionState motion;
        private btRigidBodyConstructionInfo info;
        private btRigidBody body;

        public RigidBody(btCollisionShape collisionShape, float mass, Vector3 pos) {
            shape = collisionShape;
            btTransform t = new btTransform();
            t.setIdentity();
            t.setOrigin(pos);
            Vector3 inertia = new Vector3(0.0f, 0.0f, 0.0f);
            if (mass != 0.0f) {
                shape.calculateLocalInertia(mass, inertia);
            }
            motion = new btDefaultMotionState(new Matrix4(pos, new Quaternion(), new Vector3(1.0f, 1.0f, 1.0f)));
            info = new btRigidBodyConstructionInfo(mass, motion, shape);
            body = new btRigidBody(info);
        }

        public void add(btDynamicsWorld world) {
            world.addRigidBody(body);
        }

        public void remove(btDynamicsWorld world) {
            world.removeRigidBody(body);
        }

        public void addForce(Vector3 force, Vector3 pos) {
            body.applyImpulse(force, pos);
        }

        public void addCentralForce(Vector3 force) {
            body.applyCentralImpulse(force);
        }

        public void setRotLock(boolean x, boolean y, boolean z) {
            float XRotFactor = x ? 0.0f : 1.0f;
            float YRotFactor = x ? 0.0f : 1.0f;
            float ZRotFactor = x ? 0.0f : 1.0f;
            body.setAngularFactor(new Vector3(XRotFactor, YRotFactor, ZRotFactor));
        }

        public void dispose() {
            body.dispose();
            motion.dispose();
            shape.dispose();
            info.dispose();
        }
    }

    public void createSphere(float r, float mass, Vector3 pos, String id) {
        RigidBody created = new RigidBody(new btSphereShape(r), mass, pos);
        objectList.put(id, created);
    }

    public void createCube(Vector3 dimension, float mass, Vector3 pos, String id) {
        RigidBody created = new RigidBody(new btBoxShape(dimension.scl(0.5f)), mass, pos);
        objectList.put(id, created);
    }

    public void createCapsule(float r, float h, float mass, Vector3 pos, String id) {
        RigidBody created = new RigidBody(new btCapsuleShape(r, h), mass, pos);
        objectList.put(id, created);
    }

    public void createCylinder(Vector3 dimension, float mass, Vector3 pos, String id) {
        RigidBody created = new RigidBody(new btCylinderShape(dimension.scl(0.5f)), mass, pos);
        objectList.put(id, created);
    }

    public void createCone(float r, float h, float mass, Vector3 pos, String id) {
        RigidBody created = new RigidBody(new btConeShape(r, h), mass, pos);
        objectList.put(id, created);
    }

    public void createStaticMesh(Vector3[] mesh, Vector3 pos, String id) {
        btTriangleMesh triMesh = new btTriangleMesh();
        for (int i = 0; i < mesh.length / 3; i++) {
            triMesh.addTriangle(mesh[i], mesh[i+1], mesh[i+2]);
        }
        RigidBody created = new RigidBody(new btBvhTriangleMeshShape(triMesh, true), 0.0f, pos);
        objectList.put(id, created);
    }

    public void addRigidBody(String id) {
        objectList.get(id).add(world);
    }

    public void removeRigidBodyRigidBody(String id) {
        objectList.get(id).remove(world);
    }

    public void addForce(String id, Vector3 force, Vector3 pos) {
        objectList.get(id).addForce(force, pos);
    }

    public void addCentralForce(String id, Vector3 force) {
        objectList.get(id).addCentralForce(force);
    }

    public void setRotLock(String id, boolean x, boolean y, boolean z) {
        objectList.get(id).setRotLock(x, y, z);
    }

    public void deleteRigidBody(String id) {
        objectList.get(id).dispose();
        objectList.remove(id);
    }

    public void step(float deltatime) {
        world.stepSimulation(deltatime);
    }

    public void destroy() {
        for (RigidBody object : objectList.values()) {
            object.dispose();
        }

        dispatcher.dispose();
        collisionConfig.dispose();
        solver.dispose();
        world.dispose();
        broadphase.dispose();
    }
}
