package frc.robot.utils.dashboard;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class TunableNumbers {
    private final String baseKey;
    private final Map<String, TunableNumber> numbers = new HashMap<>();

    public TunableNumbers(String baseKey, Map<String, Double> defaultValues) {
        if (defaultValues.isEmpty()) {
            throw new IllegalArgumentException("Default values map cannot be empty");
        }

        this.baseKey = baseKey;

        defaultValues.forEach(
                (key, value) -> {
                    String fullKey = baseKey + "/" + key;
                    if (Constants.IS_LIVE_DEBUG) {
                        numbers.put(key, new TunableNumber(fullKey, value));
                    }
                });
    }

    public TunableNumbers(String baseKey) {
        this.baseKey = baseKey;
        if (baseKey == null || baseKey.isEmpty()) {
            throw new IllegalArgumentException("Base key cannot be null or empty");
        }
    }

    public DoubleSupplier addNumber(String key, double defaultValue) {
        if (numbers.containsKey(key)) {
            throw new IllegalArgumentException("Key " + key + " already exists in default values map");
        }
        String fullKey = baseKey + "/" + key;
        numbers.put(key, new TunableNumber(fullKey, defaultValue));
        return () -> get(key);
    }

    public double get(String key) {
        if (!numbers.containsKey(key)) {
            throw new IllegalArgumentException("Key " + key + " not found in default values map");
        }
        if (Constants.IS_LIVE_DEBUG) {
            return numbers.get(key).get();
        }
        return numbers.get(key).get();
    }

    public double getDefault(String key) {
        if (!numbers.containsKey(key)) {
            throw new IllegalArgumentException("Key " + key + " not found in default values map");
        }
        return numbers.get(key).get();
    }

    public boolean hasChanged(int id) {
        return numbers.values().stream().anyMatch(number -> number.hasChanged(id));
    }

    public static void ifChanged(
            int id, Consumer<double[]> action, TunableNumbers... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(
                    Arrays.stream(tunableNumbers)
                            .mapToDouble(tunableNumber -> {
                                // 获取第一个值作为代表值
                                String firstKey = tunableNumber.numbers.keySet().iterator().next();
                                return tunableNumber.get(firstKey);
                            })
                            .toArray());
        }
    }

    public static void ifChanged(int id, Runnable action, TunableNumbers... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.run();
        }
    }
}
