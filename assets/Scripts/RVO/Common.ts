export class Vector2 {
    x = 0;
    y = 0;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }

    plus(vector: Vector2) {
    	return new Vector2(this.x + vector.x, this.y + vector.y);
    }
    
    minus(vector: Vector2) {
    	return new Vector2(this.x - vector.x, this.y - vector.y);
    }
    
    multiply(vector: Vector2) {
    	return this.x * vector.x + this.y * vector.y;
    }
    
    scale(k: number) {
        return new Vector2(this.x * k, this.y * k);
    }

    copy(v: Vector2) {
        this.x = v.x;
        this.y = v.y;
        return this;
    }

    clone() {
        return new Vector2(this.x, this.y);
    }

    substract(out: Vector2, other: Vector2) {
        out.x -= other.x;
        out.y -= other.y;
        return out;
    }

    lengthSqr() {
        return this.x ** 2 + this.y ** 2;
    }
}

export class Obstacle {
    next: Obstacle;
	previous: Obstacle;
	direction: Vector2;
	point: Vector2;
	id: number;
    convex: boolean;
}

export class Line {
	point: Vector2;
	direction: Vector2;
}

export class KeyValuePair<K, V> {
    key: K;
    value: V;
    constructor(key: K, value: V) {
        this.key = key;
        this.value = value;
    }
}

export class RVOMath {

    static RVO_EPSILON = 0.00001;

    static absSq(v: Vector2) {
        return v.multiply(v);
    };

    static normalize(v: Vector2) {
        return v.scale(1 / RVOMath.abs(v)); // v / abs(v)
    };

    static distSqPointLineSegment(vector1: Vector2, vector2: Vector2, vector3: Vector2) {
        let aux1 = vector3.minus(vector1);
        let aux2 = vector2.minus(vector1);
        
        let r = aux1.multiply(aux2) / RVOMath.absSq(aux2);
        
        if (r < 0) {
            return RVOMath.absSq(aux1);
        } 
        else if (r > 1) {
            return RVOMath.absSq(vector3.minus(vector2)); 
        } 
        else {
            return RVOMath.absSq(vector3.minus(vector1.plus(aux2.scale(r))));
        }
    };

    static sqr(p: number) {
        return p * p;
    };

    static det(v1: Vector2, v2: Vector2) {
        return v1.x * v2.y - v1.y * v2.x;
    };

    static abs(v: Vector2) {
        return Math.sqrt(RVOMath.absSq(v));
    };

    static leftOf(a: Vector2, b: Vector2, c: Vector2) {
        return RVOMath.det(a.minus(c), b.minus(a));
    };
    
}