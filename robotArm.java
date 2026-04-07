
public class robotArm {
    private double[] d; //arm length
    private char rot; // axis the given link rotates about
    private double[] angleLim; //- limit (index 0) and + limit (index 1) for the rotation

    private double offsetAngle, angle; // offsetAngle is start offset from theta = 0, limit is treated as +- from offset assumes default of 180, angle is current angle
    

    public robotArm(double[] d, char rot, double[] angleLim) {
        if (d == null || d.length != 3) throw new IllegalArgumentException("d must be a 3D vector");
        this.d = d;

        this.rot = (rot == 'x' || rot == 'y' || rot == 'z') ? rot : 'x';
        this.angleLim = angleLim;
        this.offsetAngle = Math.PI;
        this.angle = this.offsetAngle;
    }

    public robotArm(double[] d, char rot, double[] angleLim, double offsetAngle) {
        if (d == null || d.length != 3) throw new IllegalArgumentException("d must be a 3D vector");
        this.d = d;

        this.rot = (rot == 'x' || rot == 'y' || rot == 'z') ? rot : 'x';
        this.angleLim = angleLim;
        this.offsetAngle = offsetAngle;
        this.angle = this.offsetAngle;
    }

    public void setAngle(double angle) {
        if((offsetAngle + angleLim[0]) <= angle && (offsetAngle + angleLim[1]) >= angle) this.angle = angle;
        else throw new IllegalArgumentException("angle out of joint range");
    }

    public void setJointAllAngles(robotArm[] links, double[] jointAngles) {
        if(links.length == jointAngles.length) {
            for(int i = 0; i < links.length; i++) {
                links[i].setAngle(jointAngles[i]);
            }
        }
        else throw new IllegalArgumentException("must input the same number of angles as joints");
    }

    public double[][] Rx() {
        return new double [][] {
            {1, 0, 0,},
            {0, Math.cos(angle), -Math.sin(angle)},
            {0, Math.sin(angle), Math.cos(angle)}
        };
    }

    public double[][] Ry() {
        return new double [][] {
            {Math.cos(angle), 0, Math.sin(angle)},
            {0, 1, 0},
            {-Math.sin(angle), 0, Math.cos(angle)}
        };
    }

    public double[][] Rz() {
        return new double [][] {
            {Math.cos(angle), -Math.sin(angle), 0},
            {Math.sin(angle), Math.cos(angle), 0}, 
            {0, 0, 1}
        };
    }

    public double[][] R() {
        return switch (rot) {
            case 'x' -> Rx();
            case 'y' -> Ry();
            case 'z' -> Rz();
            default -> null;
        };
    }

    public double[][] T() {
        double[][] T = new double[4][4];
        double[][] R = R();
        T[3][0] = 0;
        T[3][1] = 0;
        T[3][2] = 0;
        T[3][3] = 1;
        
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                T[i][j] = R[i][j];
            }
        }

        for(int i = 0; i < 3; i++) {
            T[i][3] = d[i];
        }

        return T;
    }

    public static double[][] multiplyMatrix(double[][] a, double[][] b){
        int n = a.length;
        double[][] c = new double[n][n];

        for(int i = 0; i < n; i++) {
            for(int j = 0; j < n; j++) {
                for(int k = 0; k < n; k++) {
                    c[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        return c;
    }

    public static double[][] fullT(robotArm[] links) {
        double[][] full = links[0].T();

        for(int i = 0; i < (links.length - 1); i++) {
            full = multiplyMatrix(full, links[i+1].T());
        }
        return full;
    }

    public static double[][] setT(int joint, robotArm[] links) {
        if (joint < 1 || joint > links.length) throw new IllegalArgumentException("invalid joint to inspect");
        
        double[][] full = links[0].T();

        for(int i = 1; i < joint; i++) {
            full = multiplyMatrix(full, links[i].T());
        }
        return full;
    }

    public double[] getPos(int joint, robotArm[] links) {
        double[][] T = new double[4][4];
        T = (joint == links.length) ? fullT(links) : setT(joint, links);
        return new double[] {T[0][3], T[1][3], T[2][3]};
    }

    public double[][] getEndAngle(robotArm[] links) {
        double[][] T = new double[4][4], R = new double[3][3];
        T = fullT(links);
        for(int i = 0;i < 3;i++){
            for(int j = 0;j < 3;j++) {
                R[i][j] = T[i][j];
            }
        }
        return R;
    }
}