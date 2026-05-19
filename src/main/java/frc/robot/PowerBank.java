package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class PowerBank {
    private static PowerBank centralBank = new PowerBank();
    private static PowerBank testBank = new PowerBank();
    private final ArrayList<PowerAccount> accounts;
    private final ArrayList<PowerAccount> nonZeroAccounts;
    private final double maxPower = 600;
    private double remainingPower;
    private double totalOverflow;

    public PowerBank() {
        accounts = new ArrayList<PowerAccount>();
        nonZeroAccounts = new ArrayList<PowerAccount>();
    }

    public PowerAccount openAccount(String name, int priority) {
        PowerAccount acc = new PowerAccount(name, 0, 0, priority);
        accounts.add(acc);
        return acc;
    }

    public static PowerBank getInstance() {
        return centralBank;
    }

    public static PowerBank getTestInstance() {
        return testBank;
    }

    private void calculateMinimunAllocation() {
        remainingPower = maxPower;
        totalOverflow = 0;
        nonZeroAccounts.clear();

        for(PowerAccount acc: accounts) {
            acc.setAllowance(acc.getMinRequest());
            remainingPower -= acc.getMinRequest();
            if(acc.getMaxRequest() - acc.getMinRequest() > 0) {
                nonZeroAccounts.add(acc);
            }
        }

        for(PowerAccount acc: nonZeroAccounts) {
            totalOverflow += acc.getMaxRequest() - acc.getMinRequest();
        }
    }

    private void calculateExtraAllowance(ArrayList<PowerAccount> list) {
        if(totalOverflow <= remainingPower) {
             for(PowerAccount acc: list) {
             acc.setAllowance(acc.getMaxRequest());
            }
            return;
        }
        double totalPriorityInv = 0;
        for(PowerAccount acc: list) {
            totalPriorityInv += 1.0 / acc.getPriority();
        }

        ArrayList<PowerAccount> removeList = new ArrayList<>();

        for (PowerAccount acc: list) {
            double extraAllowance = acc.getMaxRequest() - acc.getMinRequest() - (totalOverflow - remainingPower)/(acc.getPriority() * totalPriorityInv);
            if (extraAllowance < 0) {
                removeList.add(acc);
                break;
            }
        }

        if (!removeList.isEmpty()) {
            list.removeAll(removeList);
            recalcOverflow(list);
            calculateExtraAllowance(list);
            return;
        }

        for (PowerAccount acc: list) {
            double extraAllowance = acc.getMaxRequest() - acc.getMinRequest() - (totalOverflow - remainingPower)/(acc.getPriority() * totalPriorityInv);
            acc.setAllowance(acc.getMinRequest() + extraAllowance);
        }
    }

    private void recalcOverflow(List<PowerAccount> list) {
        totalOverflow = 0;
        for (PowerAccount acc : list) {
            totalOverflow += acc.getMaxRequest() - acc.getMinRequest();
        }
    }

    public void calculate() {
        calculateMinimunAllocation();
        calculateExtraAllowance(nonZeroAccounts);
    }

    public List<PowerAccount> getAccounts() {
        return accounts;
    }
}