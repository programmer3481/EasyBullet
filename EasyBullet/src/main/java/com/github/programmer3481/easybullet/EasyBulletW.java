package com.github.programmer3481.easybullet;

import org.joml.*;

public class EasyBulletW {
    private EasyBullet bullet;

    public EasyBulletW() {
        bullet = new EasyBullet();
    }

    public void createSphere(float r, float mass, Vector3f pos, String id) {
        bullet.createSphere(r, mass, toGdx(pos), id);
    }

    public void createCube(Vector3f dimension, float mass, Vector3f pos, String id) {
        bullet.createCube(toGdx(dimension), mass, toGdx(pos), id);
    }

    public void createCapsule(float r, float h, float mass, Vector3f pos, String id) {
        bullet.createCapsule(r, h, mass, toGdx(pos), id);
    }

    public void createCylinder(Vector3f dimension, float mass, Vector3f pos, String id) {
        bullet.createCylinder(toGdx(dimension), mass, toGdx(pos), id);
    }

    public void createCone(float r, float h, float mass, Vector3f pos, String id) {
        bullet.createCone(r, h, mass, toGdx(pos), id);
    }

    public void addRigidBody(String id) {
        bullet.addRigidBody(id);
    }

    public void removeRigidBodyRigidBody(String id) {
        bullet.removeRigidBodyRigidBody(id);
    }

    public void addForce(String id, Vector3f force, Vector3f pos) {
        bullet.addForce(id, toGdx(force), toGdx(pos));
    }

    public void addCentralForce(String id, Vector3f force) {
        bullet.addCentralForce(id, toGdx(force));
    }

    public void setRotLock(String id, boolean x, boolean y, boolean z) {
        bullet.setRotLock(id, x, y, z);
    }

    public void deleteRigidBody(String id) {
        bullet.deleteRigidBody(id);
    }

    public void step(float deltatime) {
        bullet.step(deltatime);
    }

    public void destroy() {
        bullet.destroy();
    }

    private com.badlogic.gdx.math.Vector3 toGdx(Vector3f in) {
        float x, y, z;
        x = in.x;
        y = in.y;
        z = in.z;
        return new com.badlogic.gdx.math.Vector3(x, y, z);
    }
}
