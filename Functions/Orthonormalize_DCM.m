function C_ortho = Orthonormalize_DCM(C)

c1 = C(:,1);
c2 = C(:,2);

c1 = c1 / max(norm(c1), 1e-12);

c2 = c2 - c1 * (c1.' * c2);
c2 = c2 / max(norm(c2), 1e-12);

c3 = cross(c1, c2);
c3 = c3 / max(norm(c3), 1e-12);

C_ortho = [c1 c2 c3];

end
