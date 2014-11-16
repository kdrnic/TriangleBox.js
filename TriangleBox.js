// c is a [x, y, z] vector containing the center of the box
// e is a [x, y, z] vector containing half the dimensions of the box
// _v is a [[x0, y0, z0],[x1, y1, z1],[x2, y2, z2]] vector containing the vertices of the triangle
// translation is a [x, y, z] vector and will be set to the least translation vector that solves the collision
// this code implements the separating axis theorem for a box and a triangle
// and was written with help from the book "Real-Time Collision Detection"
// by Christer Ericson
// the main difference from the code presented in the mentioned book is the normalization of separating axes
// and calculation of the least translation vector
function TriangleBoxCollision(c, e, _v, translation)
{
	if(typeof translation == "undefined") translation = [];
	var distance = 99999, axis;
	var v0 =
	[
		_v[0][0] - c[0],
		_v[0][1] - c[1],
		_v[0][2] - c[2]
	];
	var v1 = 
	[
		_v[1][0] - c[0],
		_v[1][1] - c[1],
		_v[1][2] - c[2]
	];
	var v2 = 
	[
		_v[2][0] - c[0],
		_v[2][1] - c[1],
		_v[2][2] - c[2]
	];
	var f0 = 
	[
		v1[0] - v0[0],
		v1[1] - v0[1],
		v1[2] - v0[2]
	];
	var f1 = 
	[
		v2[0] - v1[0],
		v2[1] - v1[1],
		v2[2] - v1[2]
	];
	var f2 = 
	[
		v0[0] - v2[0],
		v0[1] - v2[1],
		v0[2] - v2[2]
	];
	var a00l = Math.sqrt(f0[2] * f0[2] + f0[1] * f0[1]);
	var a00y = -f0[2] / a00l;
	var a00z = f0[1] / a00l;
	var a00r = e[1] * Math.abs(a00y) + e[2] * Math.abs(a00z);
	var a00p0 = v0[1] * a00y + v0[2] * a00z;
	var a00p1 = v1[1] * a00y + v1[2] * a00z;
	var a00p2 = v2[1] * a00y + v2[2] * a00z;
	var a00min = Math.min(a00p0, a00p1, a00p2);
	var a00max = Math.max(a00p0, a00p1, a00p2);
	if(a00min > a00r) return false;
	if(a00max < -a00r) return false;
	else
	{
		if(a00min < -a00r)
		{
			distance = -(a00max + a00r);
		}
		else
		{
			distance = a00r - a00min;
		}
		axis = 
		[
			0,
			a00y,
			a00z
		];
	}
	var a01l = Math.sqrt(f1[2] * f1[2] + f1[1] * f1[1]);
	var a01y = -f1[2] / a01l;
	var a01z = f1[1] / a01l;
	var a01r = e[1] * Math.abs(a01y) + e[2] * Math.abs(a01z);
	var a01p0 = v0[1] * a01y + v0[2] * a01z;
	var a01p1 = v1[1] * a01y + v1[2] * a01z;
	var a01p2 = v2[1] * a01y + v2[2] * a01z;
	var a01min = Math.min(a01p0, a01p1, a01p2);
	var a01max = Math.max(a01p0, a01p1, a01p2);
	if(a01min > a01r) return false;
	if(a01max < -a01r) return false;
	else
	{
		var newDistance;
		if(a01min < -a01r)
		{
			newDistance = -(a01max + a01r);
		}
		else
		{
			newDistance = a01r - a01min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				0,
				a01y,
				a01z
			];
		}
	}
	var a02l = Math.sqrt(f2[2] * f2[2] + f2[1] * f2[1]);
	var a02y = -f2[2] / a02l;
	var a02z = f2[1] / a02l;
	var a02r = e[1] * Math.abs(a02y) + e[2] * Math.abs(a02z);
	var a02p0 = v0[1] * a02y + v0[2] * a02z;
	var a02p1 = v1[1] * a02y + v1[2] * a02z;
	var a02p2 = v2[1] * a02y + v2[2] * a02z;
	var a02min = Math.min(a02p0, a02p1, a02p2);
	var a02max = Math.max(a02p0, a02p1, a02p2);
	if(a02min > a02r) return false;
	if(a02max < -a02r) return false;
	else
	{
		var newDistance;
		if(a02min < -a02r)
		{
			newDistance = -(a02max + a02r);
		}
		else
		{
			newDistance = a02r - a02min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				0,
				a02y,
				a02z
			];
		}
	}
	var a10l = Math.sqrt(f0[2] * f0[2] + f0[0] * f0[0]);
	var a10x = f0[2] / a10l;
	var a10z = -f0[0] / a10l;
	var a10r = e[0] * Math.abs(a10x) + e[2] * Math.abs(a10z);
	var a10p0 = v0[0] * a10x + v0[2] * a10z;
	var a10p1 = v1[0] * a10x + v1[2] * a10z;
	var a10p2 = v2[0] * a10x + v2[2] * a10z;
	var a10min = Math.min(a10p0, a10p1, a10p2);
	var a10max = Math.max(a10p0, a10p1, a10p2);
	if(a10min > a10r) return false;
	if(a10max < -a10r) return false;
	else
	{
		var newDistance;
		if(a10min < -a10r)
		{
			newDistance = -(a10max + a10r);
		}
		else
		{
			newDistance = a10r - a10min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				a10x,
				0,
				a10z
			];
		}
	}
	var a11l = Math.sqrt(f1[2] * f1[2] + f1[0] * f1[0]);
	var a11x = f1[2] / a11l;
	var a11z = -f1[0] / a11l;
	var a11r = e[0] * Math.abs(a11x) + e[2] * Math.abs(a11z);
	var a11p0 = v0[0] * a11x + v0[2] * a11z;
	var a11p1 = v1[0] * a11x + v1[2] * a11z;
	var a11p2 = v2[0] * a11x + v2[2] * a11z;
	var a11min = Math.min(a11p0, a11p1, a11p2);
	var a11max = Math.max(a11p0, a11p1, a11p2);
	if(a11min > a11r) return false;
	if(a11max < -a11r) return false;
	else
	{
		var newDistance;
		if(a11min < -a11r)
		{
			newDistance = -(a11max + a11r);
		}
		else
		{
			newDistance = a11r - a11min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				a11x,
				0,
				a11z
			];
		}
	}
	var a12l = Math.sqrt(f2[2] * f2[2] + f2[0] * f2[0]);
	var a12x = f2[2] / a12l;
	var a12z = -f2[0] / a12l;
	var a12r = e[0] * Math.abs(a12x) + e[2] * Math.abs(a12z);
	var a12p0 = v0[0] * a12x + v0[2] * a12z;
	var a12p1 = v1[0] * a12x + v1[2] * a12z;
	var a12p2 = v2[0] * a12x + v2[2] * a12z;
	var a12min = Math.min(a12p0, a12p1, a12p2);
	var a12max = Math.max(a12p0, a12p1, a12p2);
	if(a12min > a12r) return false;
	if(a12max < -a12r) return false;
	else
	{
		var newDistance;
		if(a12min < -a12r)
		{
			newDistance = -(a12max + a12r);
		}
		else
		{
			newDistance = a12r - a12min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				a12x,
				0,
				a12z
			];
		}
	}
	var a20l = Math.sqrt(f0[1] * f0[1] + f0[0] * f0[0]);
	var a20x = -f0[1] / a20l;
	var a20y = f0[0] / a20l;
	var a20r = e[0] * Math.abs(a20x) + e[1] * Math.abs(a20y);
	var a20p0 = v0[0] * a20x + v0[1] * a20y;
	var a20p1 = v1[0] * a20x + v1[1] * a20y;
	var a20p2 = v2[0] * a20x + v2[1] * a20y;
	var a20min = Math.min(a20p0, a20p1, a20p2);
	var a20max = Math.max(a20p0, a20p1, a20p2);
	if(a20min > a20r) return false;
	if(a20max < -a20r) return false;
	else
	{
		var newDistance;
		if(a20min < -a20r)
		{
			newDistance = -(a20max + a20r);
		}
		else
		{
			newDistance = a20r - a20min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				a20x,
				a20y,
				0
			];
		}
	}
	var a21l = Math.sqrt(f1[1] * f1[1] + f1[0] * f1[0]);
	var a21x = -f1[1] / a21l;
	var a21y = f1[0] / a21l;
	var a21r = e[0] * Math.abs(a21x) + e[1] * Math.abs(a21y);
	var a21p0 = v0[0] * a21x + v0[1] * a21y;
	var a21p1 = v1[0] * a21x + v1[1] * a21y;
	var a21p2 = v2[0] * a21x + v2[1] * a21y;
	var a21min = Math.min(a21p0, a21p1, a21p2);
	var a21max = Math.max(a21p0, a21p1, a21p2);
	if(a21min > a21r) return false;
	if(a21max < -a21r) return false;
	else
	{
		var newDistance;
		if(a21min < -a21r)
		{
			newDistance = -(a21max + a21r);
		}
		else
		{
			newDistance = a21r - a21min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				a21x,
				a21y,
				0
			];
		}
	}
	var a22l = Math.sqrt(f2[1] * f2[1] + f2[0] * f2[0]);
	var a22x = -f2[1] / a22l;
	var a22y = f2[0] / a22l;
	var a22r = e[0] * Math.abs(a22x) + e[1] * Math.abs(a22y);
	var a22p0 = v0[0] * a22x + v0[1] * a22y;
	var a22p1 = v1[0] * a22x + v1[1] * a22y;
	var a22p2 = v2[0] * a22x + v2[1] * a22y;
	var a22min = Math.min(a22p0, a22p1, a22p2);
	var a22max = Math.max(a22p0, a22p1, a22p2);
	if(a22min > a22r) return false;
	if(a22max < -a22r) return false;
	else
	{
		var newDistance;
		if(a22min < -a22r)
		{
			newDistance = -(a22max + a22r);
		}
		else
		{
			newDistance = a22r - a22min;
		}
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				a22x,
				a22y,
				0
			];
		}
	}
	var b0max = Math.max(v0[0], v1[0], v2[0]);
	if(b0max < -e[0]) return false;
	else
	{
		var newDistance = -(e[0] + b0max);
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				1,
				0,
				0
			];
		}
	}
	var b0min = Math.min(v0[0], v1[0], v2[0]);
	if(b0min > e[0]) return false;
	else
	{
		var newDistance = b0min - e[0];
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				-1,
				0,
				0
			];
		}
	}
	var b1max = Math.max(v0[1], v1[1], v2[1]);
	if(b1max < -e[1]) return false;
	else
	{
		var newDistance = -(e[1] + b1max);
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				0,
				1,
				0
			];
		}
	}
	var b1min = Math.min(v0[1], v1[1], v2[1]);
	if(b1min > e[1]) return false;
	else
	{
		var newDistance = b1min - e[1];
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				0,
				-1,
				0
			];
		}
	}
	var b2max = Math.max(v0[2], v1[2], v2[2]);
	if(b2max < -e[2]) return false;
	else
	{
		var newDistance = -(e[2] + b2max);
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				0,
				0,
				1
			];
		}
	}
	var b2min = Math.min(v0[2], v1[2], v2[2]);
	if(b2min > e[2]) return false;
	else
	{
		var newDistance = b2min - e[2];
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				0,
				0,
				-1
			];
		}
	}
	var pn = 
	[
		-f0[2] * f1[1] + f0[1] * f1[2],
		f0[2] * f1[0] - f0[0] * f1[2],
		-f0[1] * f1[0] + f0[0] * f1[1]
	];
	var pnl = Math.sqrt(pn[0] * pn[0] + pn[1] * pn[1] + pn[2] * pn[2]);
	pn[0] /= pnl;
	pn[1] /= pnl;
	pn[2] /= pnl;
	var pd = v0[0] * pn[0] + v0[1] * pn[1] + v0[2] * pn[2];
	var pr = e[0] * Math.abs(pn[0]) + e[1] * Math.abs(pn[1]) + e[2] * Math.abs(pn[2]);
	if(Math.abs(pd) > pr) return false;
	else
	{
		var newDistance = -pr - pd;
		if(Math.abs(newDistance) < Math.abs(distance))
		{
			distance = newDistance;
			axis = 
			[
				pn[0],
				pn[1],
				pn[2]
			];
		}
	}
	distance *= -1;
	translation[0] = axis[0] * distance;
	translation[1] = axis[1] * distance;
	translation[2] = axis[2] * distance;
	return true;
}