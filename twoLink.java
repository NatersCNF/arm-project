import java.util.Arrays;

public class twoLink {
    public static void main(String[] args) {
        robotArm[] links = new robotArm[3];

        links[0] = new robotArm(new double[] {0, 0, 2}, 'y', new double[] {-Math.PI, Math.PI},(Math.PI/2));
        links[1] = new robotArm(new double[] {0, 0, 3}, 'y', new double[] {-Math.PI, Math.PI}, 0);
        links[2] = new robotArm(new double[] {0, 0, 1}, 'y', new double[] {-Math.PI, Math.PI}, 0);

        double[] startAngles = {Math.PI/4, -Math.PI/4, Math.PI/10, Math.PI/5, -Math.PI/2};
        links[0].setJointAllAngles(links, startAngles);
        System.out.println("Initial end-effector pos: " + Arrays.toString(links[0].getPos(links.length, links)));

        double[] endAngles = {-Math.PI/2, -Math.PI/2, Math.PI/5, -Math.PI/2, Math.PI/5};
        links[0].setJointAllAngles(links, endAngles);
        System.out.println("Secondary end-effector pos: " + Arrays.toString(links[0].getPos(links.length, links)));
    }
}