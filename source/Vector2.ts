export class Vector2 {
    x: number;
    y: number;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }

    moins(v: Vector2): Vector2 {
        return new Vector2(this.x - v.x, this.y - v.y);
    }

    moinsSelf(): Vector2 {
        return new Vector2(-this.x, -this.y);
    }

    plus(v: Vector2): Vector2 {
        return new Vector2(this.x + v.x, this.y + v.y);
    }

    mul(v: Vector2): number {
        return this.x * v.x + this.y * v.y;

    }

    mul_k(k: number): Vector2 {
        return new Vector2(this.x * k, this.y * k);
    }

    div_k(k: number): Vector2 {
        let s: number = 1 / k;
        return new Vector2(this.x * s, this.y * s);
    }

    static absSq(v: Vector2): number {
        return v.mul(v);
    }

    static abs(v: Vector2): number {
        return Math.sqrt(v.mul(v));
    }

    static det(v1: Vector2, v2: Vector2): number {
        return v1.x * v2.y - v1.y * v2.x;
    }

    static normalize(v: Vector2): Vector2 {
        return v.div_k(Vector2.abs(v));
    }

    static leftOf(a: Vector2, b: Vector2, c: Vector2): number {
        return Vector2.det(a.moins(c), b.moins(a));
    }

    static distSqPointLineSegment(a: Vector2, b: Vector2, c: Vector2): number {
        let r: number = c.moins(a).mul(b.moins(a)) / Vector2.absSq(b.moins(a));
        if (r < 0) {
            return Vector2.absSq(c.moins(a));
        } else if (r > 1) {
            return Vector2.absSq(c.moins(b));
        }
        return Vector2.absSq(c.moins(a.plus(b.moins(a).mul_k(r))));
    }
}