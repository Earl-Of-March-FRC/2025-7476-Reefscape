import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// Sanity test to check if JUnit is working
class SanityTest {
    private int x;

    @BeforeEach
    void setup() {
        x = 1;
    }

    @AfterEach
    void shutdown() {
    }

    @Test
    void passingTest() {
        assertTrue(true);
    }

    @Test
    void setupTest() {
        assertEquals(x, 1);
    }

}
